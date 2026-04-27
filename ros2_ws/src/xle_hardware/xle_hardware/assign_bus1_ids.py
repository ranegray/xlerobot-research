"""Interactive one-motor-at-a-time ID assignment for bus1 motors.

Use only when scan_bus1 reports the wrong layout, or when bringing up new
motors (e.g. the right arm bus2 once the same flow is reused there).

Required wiring:
    - Bus controller plugged in (default /dev/ttyACM0)
    - 12V supply ON
    - The motor being assigned is the ONLY motor on the chain, off the arm,
      on a single short cable
    - All other motors physically disconnected from the bus

For each motor in turn, the script:
    1. Prompts the operator to plug in only that motor and press enter
    2. Scans the configured baud rate, then a recovery list, broadcast-pings
       to find whatever ID the motor currently has
    3. Verifies model number is 777 (STS3215)
    4. Disables torque, unlocks EEPROM
    5. Writes the target ID and target baud rate
    6. Re-locks EEPROM
    7. Re-pings at the new ID/baud to confirm

Usage:
    ros2 run xle_hardware assign_bus1_ids
    ros2 run xle_hardware assign_bus1_ids --only 7,8         # head only
    ros2 run xle_hardware assign_bus1_ids --port /dev/ttyACM1
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, List, Optional, Tuple

from xle_hardware._sdk import load_sdk

scs = load_sdk()

from xle_hardware.bus1_layout import (
    ADDR_BAUD_RATE,
    ADDR_ID,
    ADDR_LOCK,
    ADDR_MODEL_NUMBER,
    ADDR_TORQUE_ENABLE,
    BAUDRATE_INDEX,
    BUS1_MOTORS,
    Bus1Motor,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    DEFAULT_PROTOCOL_VERSION,
    STS3215_MODEL_NUMBER,
)


RECOVERY_BAUDRATES: Tuple[int, ...] = (
    1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400,
)


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Assign bus1 motor IDs one at a time.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--target-baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--protocol-version", type=int, default=DEFAULT_PROTOCOL_VERSION)
    parser.add_argument(
        "--only",
        default="",
        help="Comma-separated list of target IDs to assign (default: all 8).",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip confirmation prompts before each write (use with care).",
    )
    return parser.parse_args(argv)


def select_motors(only: str) -> List[Bus1Motor]:
    if not only:
        return list(BUS1_MOTORS)
    wanted = {int(x) for x in only.split(",") if x.strip()}
    selected = [m for m in BUS1_MOTORS if m.motor_id in wanted]
    missing = wanted - {m.motor_id for m in selected}
    if missing:
        raise SystemExit(f"--only contained unknown IDs: {sorted(missing)}")
    return selected


def open_port(port_path: str) -> scs.PortHandler:
    handler = scs.PortHandler(port_path)
    if not handler.openPort():
        raise SystemExit(f"could not open serial port {port_path!r}")
    return handler


def discover_present_motor(
    port: scs.PortHandler, packet: scs.PacketHandler
) -> Optional[Tuple[int, int, int]]:
    """Find the single motor on the bus.

    Returns (current_id, current_baudrate, model_number) or None.
    """
    for baudrate in RECOVERY_BAUDRATES:
        if not port.setBaudRate(baudrate):
            continue
        time.sleep(0.05)
        for motor_id in range(1, 31):
            model_number, comm, _ = packet.ping(port, motor_id)
            if comm == scs.COMM_SUCCESS:
                return motor_id, baudrate, model_number
    return None


def write1(
    port: scs.PortHandler, packet: scs.PacketHandler, motor_id: int, address: int, value: int
) -> bool:
    comm, err = packet.write1ByteTxRx(port, motor_id, address, value)
    return comm == scs.COMM_SUCCESS and err == 0


def write_id_to_eeprom(
    port: scs.PortHandler,
    packet: scs.PacketHandler,
    current_id: int,
    target_id: int,
    target_baudrate: int,
) -> None:
    if not write1(port, packet, current_id, ADDR_TORQUE_ENABLE, 0):
        raise RuntimeError(f"failed to disable torque on id={current_id}")
    if not write1(port, packet, current_id, ADDR_LOCK, 0):
        raise RuntimeError(f"failed to unlock EEPROM on id={current_id}")
    if not write1(port, packet, current_id, ADDR_ID, target_id):
        raise RuntimeError(f"failed to write new ID {target_id} to id={current_id}")
    baud_index = BAUDRATE_INDEX.get(target_baudrate)
    if baud_index is None:
        raise RuntimeError(f"unsupported target baudrate {target_baudrate}")
    if not write1(port, packet, target_id, ADDR_BAUD_RATE, baud_index):
        raise RuntimeError(f"failed to write baud index on new id={target_id}")
    if not write1(port, packet, target_id, ADDR_LOCK, 1):
        raise RuntimeError(f"failed to re-lock EEPROM on new id={target_id}")


def confirm_at(
    port: scs.PortHandler,
    packet: scs.PacketHandler,
    target_id: int,
    target_baudrate: int,
) -> bool:
    if not port.setBaudRate(target_baudrate):
        return False
    time.sleep(0.05)
    model_number, comm, _ = packet.ping(port, target_id)
    return comm == scs.COMM_SUCCESS and model_number == STS3215_MODEL_NUMBER


def assign_one(
    port: scs.PortHandler,
    packet: scs.PacketHandler,
    motor: Bus1Motor,
    target_baudrate: int,
    skip_prompt: bool,
) -> None:
    print()
    print(f"--- {motor.joint_name} (target ID {motor.motor_id}, role: {motor.role}) ---")
    input(
        f"Connect ONLY the {motor.role} motor to the bus controller, "
        f"power on, then press enter."
    )

    discovered = discover_present_motor(port, packet)
    if discovered is None:
        raise RuntimeError(
            "no motor found on the bus. check the cable, that the motor is the only "
            "one connected, and that 12 V is on."
        )

    current_id, current_baudrate, model_number = discovered
    print(
        f"  found id={current_id} baud={current_baudrate} model={model_number}"
    )

    if model_number != STS3215_MODEL_NUMBER:
        raise RuntimeError(
            f"model number {model_number} != {STS3215_MODEL_NUMBER}. "
            f"this is not an STS3215. abort and physically remove this motor."
        )

    if current_id == motor.motor_id and current_baudrate == target_baudrate:
        print("  already at target ID and baud, skipping write.")
        return

    if not skip_prompt:
        proceed = input(
            f"  write id {current_id}@{current_baudrate} -> "
            f"{motor.motor_id}@{target_baudrate}? [y/N] "
        )
        if proceed.strip().lower() not in {"y", "yes"}:
            print("  skipped.")
            return

    write_id_to_eeprom(port, packet, current_id, motor.motor_id, target_baudrate)

    if confirm_at(port, packet, motor.motor_id, target_baudrate):
        print(f"  OK  motor now responds at id={motor.motor_id} baud={target_baudrate}")
    else:
        raise RuntimeError(
            f"wrote new ID/baud but motor is not responding at "
            f"id={motor.motor_id} baud={target_baudrate}. power-cycle and re-scan."
        )


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else list(argv))

    motors = select_motors(args.only)
    print(
        f"will assign {len(motors)} motor(s) on {args.port} "
        f"at target baud {args.target_baudrate}: "
        f"{[(m.motor_id, m.joint_name) for m in motors]}"
    )
    print(
        "before proceeding: every other bus1 motor must be physically "
        "disconnected from the controller. each motor is brought up alone, on "
        "the bench, off the arm."
    )
    if not args.yes:
        proceed = input("ready? [y/N] ")
        if proceed.strip().lower() not in {"y", "yes"}:
            print("aborted.")
            return 0

    port = open_port(args.port)
    packet = scs.PacketHandler(args.protocol_version)
    failures: List[str] = []
    try:
        for motor in motors:
            try:
                assign_one(port, packet, motor, args.target_baudrate, args.yes)
            except RuntimeError as exc:
                print(f"  FAILED  {exc}", file=sys.stderr)
                failures.append(motor.joint_name)
    finally:
        port.closePort()

    print()
    if failures:
        print(f"done with {len(failures)} failure(s): {failures}", file=sys.stderr)
        return 1
    print("done. now reconnect the full daisy chain and run scan_bus1 to confirm.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
