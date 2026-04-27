"""Interactive bus1 calibration capture.

UX modeled on lerobot's flow: one ENTER for homing, one ENTER to start range
capture, ENTER again to stop. Total operator presses: ~3, regardless of how
many motors.

Workflow:

    1. Disable torque on all 8 motors so they back-drive freely.
    2. Operator manually poses the arm at chosen URDF zero pose, presses ENTER.
       Current raw step value of every motor is saved as homing_offset_steps.
    3. Operator presses ENTER to start range capture, then wiggles every joint
       through its full range simultaneously (or sequentially, doesn't matter).
       A live table shows MIN/POS/MAX per joint, updating in place. Press ENTER
       to stop. Final mins/maxes are saved as raw_min/raw_max.
    4. Writes YAML to ~/.xle/bus1_calibration.yaml (overridable with --output).

Re-run any time motors are remounted, the arm is rebuilt, or the URDF changes
the joint zero conventions.

Usage:
    ros2 run xle_hardware calibrate_bus1
    ros2 run xle_hardware calibrate_bus1 --only Rotation_L,Pitch_L
    ros2 run xle_hardware calibrate_bus1 --output ./my_cal.yaml
    ros2 run xle_hardware calibrate_bus1 --no-range          # skip step 3
"""

from __future__ import annotations

import argparse
import datetime as dt
import select
import socket
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

from xle_hardware._sdk import load_sdk

scs = load_sdk()

from xle_hardware.bus1_layout import (
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    BUS1_MOTORS,
    BY_JOINT,
    Bus1Motor,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    DEFAULT_PROTOCOL_VERSION,
    STS3215_MODEL_NUMBER,
    STS3215_RESOLUTION,
)


SCHEMA = "xle.bus1_calibration.v0"
DEFAULT_OUTPUT = Path.home() / ".xle" / "bus1_calibration.yaml"
PRESENT_POSITION_LEN = 2
LIVE_REFRESH_S = 0.05  # ~20 Hz


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture bus1 calibration interactively.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--protocol-version", type=int, default=DEFAULT_PROTOCOL_VERSION)
    parser.add_argument(
        "--only",
        default="",
        help="Comma-separated joint names to calibrate. Default: all 8.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Output YAML path (default: {DEFAULT_OUTPUT}).",
    )
    parser.add_argument(
        "--no-range",
        action="store_true",
        help="Skip the range capture step (homing offset only).",
    )
    return parser.parse_args(argv)


def select_motors(only: str) -> List[Bus1Motor]:
    if not only:
        return list(BUS1_MOTORS)
    wanted = {name.strip() for name in only.split(",") if name.strip()}
    unknown = wanted - set(BY_JOINT)
    if unknown:
        raise SystemExit(f"--only contained unknown joints: {sorted(unknown)}")
    return [m for m in BUS1_MOTORS if m.joint_name in wanted]


def open_port(port_path: str, baudrate: int) -> scs.PortHandler:
    handler = scs.PortHandler(port_path)
    if not handler.openPort():
        raise SystemExit(f"could not open {port_path}")
    if not handler.setBaudRate(baudrate):
        handler.closePort()
        raise SystemExit(f"could not set baudrate {baudrate}")
    return handler


def write_byte(port, packet, motor_id, address, value) -> None:
    comm, err = packet.write1ByteTxRx(port, motor_id, address, value)
    if comm != scs.COMM_SUCCESS or err != 0:
        raise RuntimeError(
            f"write byte addr={address} on id={motor_id} failed (comm={comm}, err={err})"
        )


def torque_off_all(port, packet, motors: List[Bus1Motor]) -> None:
    for m in motors:
        try:
            write_byte(port, packet, m.motor_id, ADDR_TORQUE_ENABLE, 0)
        except RuntimeError as exc:
            print(f"  WARN  could not disable torque on {m.joint_name}: {exc}", file=sys.stderr)


def verify_layout(port, packet, motors: List[Bus1Motor]) -> None:
    for m in motors:
        model_number, comm, _ = packet.ping(port, m.motor_id)
        if comm != scs.COMM_SUCCESS:
            raise SystemExit(f"id {m.motor_id} ({m.joint_name}) did not respond to ping")
        if model_number != STS3215_MODEL_NUMBER:
            raise SystemExit(
                f"id {m.motor_id} reports model {model_number}, expected "
                f"{STS3215_MODEL_NUMBER}. abort."
            )


def make_sync_read(port, packet, motors: List[Bus1Motor]) -> scs.GroupSyncRead:
    reader = scs.GroupSyncRead(port, packet, ADDR_PRESENT_POSITION, PRESENT_POSITION_LEN)
    for m in motors:
        if not reader.addParam(m.motor_id):
            raise RuntimeError(f"GroupSyncRead.addParam failed for id={m.motor_id}")
    return reader


def sync_read_positions(reader: scs.GroupSyncRead, motors: List[Bus1Motor]) -> Dict[str, int]:
    comm = reader.txRxPacket()
    if comm != scs.COMM_SUCCESS:
        raise RuntimeError(f"GroupSyncRead.txRxPacket failed (comm={comm})")
    out: Dict[str, int] = {}
    for m in motors:
        if not reader.isAvailable(m.motor_id, ADDR_PRESENT_POSITION, PRESENT_POSITION_LEN):
            raise RuntimeError(f"GroupSyncRead missing data for id={m.motor_id}")
        out[m.joint_name] = reader.getData(m.motor_id, ADDR_PRESENT_POSITION, PRESENT_POSITION_LEN)
    return out


def enter_pressed_nonblocking() -> bool:
    """True iff a complete line is available on stdin and it's empty (just ENTER)."""
    if not select.select([sys.stdin], [], [], 0)[0]:
        return False
    return sys.stdin.readline().strip() == ""


def move_cursor_up(lines: int) -> None:
    print(f"\033[{lines}A", end="", flush=True)


def capture_homing(reader, motors: List[Bus1Motor]) -> Dict[str, int]:
    print()
    print("=" * 60)
    print("STEP 1 / HOMING OFFSET")
    print("=" * 60)
    print(
        "Pose the arm in the configuration you want to call URDF zero.\n"
        "  - Typical: arm extended forward, gripper open, head looking forward.\n"
        "  - The current pose becomes the reference (0 rad on every joint).\n"
    )
    input("press ENTER when the arm is at zero pose ")
    homing = sync_read_positions(reader, motors)
    print()
    print(f"  {'joint':<18s}  raw step")
    for m in motors:
        print(f"  {m.joint_name:<18s}  {homing[m.joint_name]:4d}")
    return homing


def capture_range(reader, motors: List[Bus1Motor]) -> Dict[str, Dict[str, int]]:
    print()
    print("=" * 60)
    print("STEP 2 / RANGE CAPTURE")
    print("=" * 60)
    print(
        "Move EVERY joint through its full physical range.\n"
        "  - You can do them all at once or one-by-one; the live table shows\n"
        "    each joint's running MIN, current POS, and MAX.\n"
        "  - Press ENTER to stop when every joint has hit both extremes.\n"
    )
    input("press ENTER to start ")

    start = sync_read_positions(reader, motors)
    mins = dict(start)
    maxes = dict(start)

    header = f"  {'joint':<18s}  {'MIN':>5}  {'POS':>5}  {'MAX':>5}  {'span':>5}"
    print(header)
    for m in motors:
        print(f"  {m.joint_name:<18s}  {start[m.joint_name]:>5}  {start[m.joint_name]:>5}  {start[m.joint_name]:>5}  {0:>5}")
    print("press ENTER to stop ")

    table_height = len(motors) + 1  # rows + the "press enter" line

    while True:
        try:
            positions = sync_read_positions(reader, motors)
        except RuntimeError as exc:
            print(f"\nread error during range capture: {exc}", file=sys.stderr)
            break

        for m in motors:
            joint = m.joint_name
            pos = positions[joint]
            if pos < mins[joint]:
                mins[joint] = pos
            if pos > maxes[joint]:
                maxes[joint] = pos

        move_cursor_up(table_height)
        for m in motors:
            joint = m.joint_name
            span = maxes[joint] - mins[joint]
            print(
                f"  {joint:<18s}  {mins[joint]:>5}  {positions[joint]:>5}  "
                f"{maxes[joint]:>5}  {span:>5}",
                flush=True,
            )
        print("press ENTER to stop ", flush=True)

        if enter_pressed_nonblocking():
            break
        time.sleep(LIVE_REFRESH_S)

    return {
        m.joint_name: {"raw_min": mins[m.joint_name], "raw_max": maxes[m.joint_name]}
        for m in motors
    }


def write_yaml(
    output: Path,
    motors: List[Bus1Motor],
    homing: Dict[str, int],
    range_capture: Optional[Dict[str, Dict[str, int]]],
    port_path: str,
    baudrate: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    timestamp = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")
    lines: List[str] = []
    lines.append(f"schema: {SCHEMA}")
    lines.append(f"calibrated_at: '{timestamp}'")
    lines.append(f"host: {socket.gethostname()}")
    lines.append(f"port: {port_path}")
    lines.append(f"baudrate: {baudrate}")
    lines.append(f"resolution_steps: {STS3215_RESOLUTION}")
    lines.append("motors:")
    for m in motors:
        lines.append(f"  {m.joint_name}:")
        lines.append(f"    motor_id: {m.motor_id}")
        lines.append(f"    sign: {m.default_sign}")
        lines.append(f"    homing_offset_steps: {homing[m.joint_name]}")
        if range_capture is not None and m.joint_name in range_capture:
            r = range_capture[m.joint_name]
            lines.append(f"    raw_min: {r['raw_min']}")
            lines.append(f"    raw_max: {r['raw_max']}")
        else:
            lines.append("    raw_min: null")
            lines.append("    raw_max: null")
    output.write_text("\n".join(lines) + "\n")


def warn_if_zero_outside_range(
    motors: List[Bus1Motor],
    homing: Dict[str, int],
    range_capture: Optional[Dict[str, Dict[str, int]]],
) -> None:
    if range_capture is None:
        return
    for m in motors:
        h = homing[m.joint_name]
        rng = range_capture[m.joint_name]
        if not (rng["raw_min"] <= h <= rng["raw_max"]):
            print(
                f"WARNING  {m.joint_name}: zero pose ({h}) outside captured range "
                f"[{rng['raw_min']}, {rng['raw_max']}]. URDF zero may be unreachable.",
                file=sys.stderr,
            )
        elif rng["raw_max"] - rng["raw_min"] < 50:
            print(
                f"WARNING  {m.joint_name}: range only {rng['raw_max'] - rng['raw_min']} "
                f"steps (<2 deg). Suspiciously narrow.",
                file=sys.stderr,
            )


def main(argv=None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else list(argv))
    motors = select_motors(args.only)
    print(
        f"calibrating {len(motors)} motor(s): "
        f"{[m.joint_name for m in motors]}"
    )
    print(f"output: {args.output}")
    if args.output.exists():
        confirm = input(f"  output exists. overwrite? [y/N] ")
        if confirm.strip().lower() not in {"y", "yes"}:
            print("aborted.")
            return 0

    port = open_port(args.port, args.baudrate)
    packet = scs.PacketHandler(args.protocol_version)
    try:
        verify_layout(port, packet, motors)
        torque_off_all(port, packet, motors)
        print("torque disabled. arm should now back-drive freely.")
        reader = make_sync_read(port, packet, motors)
        homing = capture_homing(reader, motors)
        range_capture = None if args.no_range else capture_range(reader, motors)
        warn_if_zero_outside_range(motors, homing, range_capture)
        write_yaml(args.output, motors, homing, range_capture, args.port, args.baudrate)
    finally:
        try:
            torque_off_all(port, packet, motors)
        except Exception:  # noqa: BLE001
            pass
        port.closePort()

    print()
    print(f"OK  wrote {args.output}")
    print("next: ros2 launch xle_bringup view_real_arm.launch.py enable_torque:=true")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
