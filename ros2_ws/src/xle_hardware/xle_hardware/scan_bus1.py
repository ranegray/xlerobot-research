"""Read-only bus1 motor discovery.

Pings every motor on the bus, reports model/firmware/position/voltage/
temperature, and compares against the expected layout in bus1_layout.py.

Sends only Ping and Read packets. Never enables torque, never writes.
Safe to run at any time the bus is powered.

Usage:
    ros2 run xle_hardware scan_bus1
    ros2 run xle_hardware scan_bus1 --port /dev/ttyACM1 --baudrate 1000000

Exit codes:
    0  expected layout found exactly (IDs 1-8 with model 777)
    1  bus opened but layout does not match expectation
    2  could not open the serial port
"""

from __future__ import annotations

import argparse
import sys
from typing import Dict, List, Tuple

from xle_hardware._sdk import load_sdk

scs = load_sdk()

from xle_hardware.bus1_layout import (
    ADDR_FIRMWARE_MAJOR,
    ADDR_FIRMWARE_MINOR,
    ADDR_PRESENT_POSITION,
    ADDR_PRESENT_TEMPERATURE,
    ADDR_PRESENT_VOLTAGE,
    BY_ID,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    DEFAULT_PROTOCOL_VERSION,
    EXPECTED_IDS,
    STS3215_MODEL_NUMBER,
)


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bus1 motor discovery (read-only).")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument(
        "--protocol-version",
        type=int,
        default=DEFAULT_PROTOCOL_VERSION,
        help="Feetech protocol version. STS3215 is 0.",
    )
    return parser.parse_args(argv)


def open_port(port: str, baudrate: int) -> scs.PortHandler:
    handler = scs.PortHandler(port)
    if not handler.openPort():
        raise OSError(f"Could not open serial port {port!r}.")
    if not handler.setBaudRate(baudrate):
        handler.closePort()
        raise OSError(f"Could not set baudrate {baudrate} on {port!r}.")
    return handler


def ping_all(port: scs.PortHandler, packet: scs.PacketHandler) -> Dict[int, int]:
    found: Dict[int, int] = {}
    for motor_id in range(1, 31):
        model_number, comm, _ = packet.ping(port, motor_id)
        if comm == scs.COMM_SUCCESS:
            found[motor_id] = model_number
    return found


def read_byte(
    port: scs.PortHandler, packet: scs.PacketHandler, motor_id: int, address: int
) -> int | None:
    value, comm, _ = packet.read1ByteTxRx(port, motor_id, address)
    return value if comm == scs.COMM_SUCCESS else None


def read_word(
    port: scs.PortHandler, packet: scs.PacketHandler, motor_id: int, address: int
) -> int | None:
    value, comm, _ = packet.read2ByteTxRx(port, motor_id, address)
    return value if comm == scs.COMM_SUCCESS else None


def collect(
    port: scs.PortHandler, packet: scs.PacketHandler, found: Dict[int, int]
) -> List[Tuple[int, Dict[str, object]]]:
    rows = []
    for motor_id, model_number in sorted(found.items()):
        major = read_byte(port, packet, motor_id, ADDR_FIRMWARE_MAJOR)
        minor = read_byte(port, packet, motor_id, ADDR_FIRMWARE_MINOR)
        position = read_word(port, packet, motor_id, ADDR_PRESENT_POSITION)
        voltage = read_byte(port, packet, motor_id, ADDR_PRESENT_VOLTAGE)
        temperature = read_byte(port, packet, motor_id, ADDR_PRESENT_TEMPERATURE)
        rows.append(
            (
                motor_id,
                {
                    "model_number": model_number,
                    "firmware": f"{major}.{minor}" if major is not None else "?",
                    "position_steps": position,
                    "voltage_v": voltage / 10.0 if voltage is not None else None,
                    "temperature_c": temperature,
                    "expected_joint": BY_ID[motor_id].joint_name if motor_id in BY_ID else None,
                    "expected_role": BY_ID[motor_id].role if motor_id in BY_ID else None,
                },
            )
        )
    return rows


def print_report(rows: List[Tuple[int, Dict[str, object]]], found_ids: List[int]) -> None:
    print()
    print(f"{'ID':>3}  {'model':>5}  {'fw':>5}  {'pos':>5}  {'V':>5}  {'°C':>3}  joint")
    print("-" * 60)
    for motor_id, info in rows:
        joint = info["expected_joint"] or "(unmapped)"
        role = info["expected_role"] or ""
        pos = info["position_steps"]
        volt = info["voltage_v"]
        temp = info["temperature_c"]
        print(
            f"{motor_id:>3}  "
            f"{info['model_number']:>5}  "
            f"{info['firmware']:>5}  "
            f"{pos if pos is not None else '?':>5}  "
            f"{volt if volt is not None else '?':>5}  "
            f"{temp if temp is not None else '?':>3}  "
            f"{joint} ({role})"
        )

    expected = set(EXPECTED_IDS)
    missing = sorted(expected - set(found_ids))
    unexpected = sorted(set(found_ids) - expected)
    wrong_model = [mid for mid, info in rows if info["model_number"] != STS3215_MODEL_NUMBER]

    print()
    if not missing and not unexpected and not wrong_model:
        print("OK  bus1 matches expected layout (IDs 1-8, all STS3215).")
    else:
        if missing:
            joints = [BY_ID[mid].joint_name for mid in missing]
            print(f"MISSING  expected IDs not on bus: {missing} ({joints})")
        if unexpected:
            print(f"UNEXPECTED  IDs found that are not in the layout: {unexpected}")
        if wrong_model:
            print(
                f"WRONG MODEL  IDs reporting model number != {STS3215_MODEL_NUMBER}: "
                f"{wrong_model}"
            )


def main(argv: List[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    try:
        port = open_port(args.port, args.baudrate)
    except OSError as exc:
        print(f"ERROR  {exc}", file=sys.stderr)
        return 2

    packet = scs.PacketHandler(args.protocol_version)
    print(f"scanning {args.port} at {args.baudrate} baud, protocol v{args.protocol_version}...")
    try:
        found = ping_all(port, packet)
        if not found:
            print(
                "no motors responded to ping. check power, cabling, baud rate, "
                "and that no other process holds the serial port.",
                file=sys.stderr,
            )
            return 1
        rows = collect(port, packet, found)
    finally:
        port.closePort()

    print_report(rows, list(found.keys()))

    expected = set(EXPECTED_IDS)
    found_ids = set(found.keys())
    if found_ids == expected and all(
        info["model_number"] == STS3215_MODEL_NUMBER for _, info in rows
    ):
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
