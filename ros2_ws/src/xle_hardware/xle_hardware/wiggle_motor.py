"""Single-motor wiggle test — the smallest safe physical-motion proof.

Picks one bus1 motor, reads its current raw position, enables torque, moves
+N steps then -N steps then returns to start, disables torque. No calibration
needed. Default 100 steps ≈ 8.8° at the motor.

Use this instead of the full bridge when you just want to confirm the bus
controls torque correctly without touching the arm at scale.

Examples:
    # Gripper: low load, short lever arm.
    ros2 run xle_hardware wiggle_motor --joint Jaw_L

    # Head pan: low load.
    ros2 run xle_hardware wiggle_motor --id 7 --steps 60

    # Shoulder pan: larger lever arm, keep steps small.
    ros2 run xle_hardware wiggle_motor --id 1 --steps 50

Safety:
    - Bridge motor by ID/joint, not all at once.
    - Step delta is clamped to [10, 200] (≈0.9°-17.6°).
    - Acceleration is forced to a slow value before motion.
    - Torque is disabled on exit, even on Ctrl-C.
    - Refuses to run if the resulting target would exceed the 0..4095 range.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List

from xle_hardware._sdk import load_sdk

scs = load_sdk()

from xle_hardware.bus1_layout import (
    ADDR_ACCELERATION,
    ADDR_GOAL_POSITION,
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    BY_ID,
    BY_JOINT,
    Bus1Motor,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    DEFAULT_PROTOCOL_VERSION,
    STS3215_MODEL_NUMBER,
    STS3215_RESOLUTION,
)

SAFE_ACCELERATION = 20  # 0..254, smaller is slower
DEFAULT_DWELL_S = 1.5


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Wiggle a single bus1 motor.")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--id", type=int, help="Motor ID (1-8).")
    target.add_argument("--joint", type=str, help="Joint name, e.g. Jaw_L.")
    parser.add_argument(
        "--steps",
        type=int,
        default=100,
        help="Raw-step delta for the wiggle (clamped to 10..200).",
    )
    parser.add_argument("--dwell", type=float, default=DEFAULT_DWELL_S)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--protocol-version", type=int, default=DEFAULT_PROTOCOL_VERSION)
    return parser.parse_args(argv)


def resolve_motor(args: argparse.Namespace) -> Bus1Motor:
    if args.id is not None:
        if args.id not in BY_ID:
            raise SystemExit(f"--id {args.id} is not in the bus1 layout {sorted(BY_ID)}.")
        return BY_ID[args.id]
    if args.joint not in BY_JOINT:
        raise SystemExit(
            f"--joint {args.joint!r} is not in the bus1 layout {sorted(BY_JOINT)}."
        )
    return BY_JOINT[args.joint]


def write_byte(port, packet, motor_id, address, value) -> None:
    comm, err = packet.write1ByteTxRx(port, motor_id, address, value)
    if comm != scs.COMM_SUCCESS or err != 0:
        raise RuntimeError(
            f"write byte addr={address} val={value} on id={motor_id} failed "
            f"(comm={comm}, err={err})"
        )


def write_word(port, packet, motor_id, address, value) -> None:
    comm, err = packet.write2ByteTxRx(port, motor_id, address, value)
    if comm != scs.COMM_SUCCESS or err != 0:
        raise RuntimeError(
            f"write word addr={address} val={value} on id={motor_id} failed "
            f"(comm={comm}, err={err})"
        )


def read_word(port, packet, motor_id, address) -> int:
    value, comm, err = packet.read2ByteTxRx(port, motor_id, address)
    if comm != scs.COMM_SUCCESS:
        raise RuntimeError(
            f"read word addr={address} on id={motor_id} failed (comm={comm}, err={err})"
        )
    return value


def goto_and_wait(port, packet, motor_id, target_steps, dwell_s) -> None:
    write_word(port, packet, motor_id, ADDR_GOAL_POSITION, target_steps)
    time.sleep(dwell_s)
    actual = read_word(port, packet, motor_id, ADDR_PRESENT_POSITION)
    print(f"  goal={target_steps:4d}  actual={actual:4d}  err={target_steps - actual:+d}")


def main(argv=None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else list(argv))
    motor = resolve_motor(args)
    steps = max(10, min(200, args.steps))
    if steps != args.steps:
        print(f"clamped --steps {args.steps} to {steps}", file=sys.stderr)

    port = scs.PortHandler(args.port)
    if not port.openPort():
        raise SystemExit(f"could not open port {args.port}")
    if not port.setBaudRate(args.baudrate):
        port.closePort()
        raise SystemExit(f"could not set baudrate {args.baudrate}")

    packet = scs.PacketHandler(args.protocol_version)
    print(
        f"wiggle id={motor.motor_id} joint={motor.joint_name} "
        f"role='{motor.role}' steps=±{steps}"
    )

    try:
        model_number, comm, _ = packet.ping(port, motor.motor_id)
        if comm != scs.COMM_SUCCESS:
            raise SystemExit(f"id {motor.motor_id} did not respond to ping")
        if model_number != STS3215_MODEL_NUMBER:
            raise SystemExit(
                f"id {motor.motor_id} reports model {model_number}, expected "
                f"{STS3215_MODEL_NUMBER}. abort."
            )

        start = read_word(port, packet, motor.motor_id, ADDR_PRESENT_POSITION)
        target_up = start + steps
        target_down = start - steps
        if not (0 <= target_down and target_up <= STS3215_RESOLUTION - 1):
            raise SystemExit(
                f"start={start}, ±{steps} would exit valid range "
                f"[0, {STS3215_RESOLUTION - 1}]. choose smaller --steps."
            )

        print(f"  start position: {start} (will move to {target_up}, then {target_down}, then {start})")
        write_byte(port, packet, motor.motor_id, ADDR_ACCELERATION, SAFE_ACCELERATION)

        confirm = input("enable torque and start the wiggle? [y/N] ")
        if confirm.strip().lower() not in {"y", "yes"}:
            print("aborted before torque enable.")
            return 0

        write_byte(port, packet, motor.motor_id, ADDR_TORQUE_ENABLE, 1)
        try:
            print("  + UP")
            goto_and_wait(port, packet, motor.motor_id, target_up, args.dwell)
            print("  - DOWN")
            goto_and_wait(port, packet, motor.motor_id, target_down, args.dwell)
            print("  = HOME")
            goto_and_wait(port, packet, motor.motor_id, start, args.dwell)
        finally:
            try:
                write_byte(port, packet, motor.motor_id, ADDR_TORQUE_ENABLE, 0)
                print("torque disabled.")
            except RuntimeError as exc:
                print(f"WARNING failed to disable torque: {exc}", file=sys.stderr)
    finally:
        port.closePort()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
