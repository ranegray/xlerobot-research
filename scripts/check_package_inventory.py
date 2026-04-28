#!/usr/bin/env python3
from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "ros2_ws" / "src"


@dataclass(frozen=True)
class PackageRecord:
    name: str
    path: Path
    build_type: str
    description: str


def _package_name(package_xml: Path) -> str:
    tree = ET.parse(package_xml)
    name = tree.findtext("name")
    if not name:
        raise ValueError(f"{package_xml} is missing <name>")
    return name


def _build_type(package_xml: Path) -> str:
    tree = ET.parse(package_xml)
    build_type = tree.findtext("export/build_type")
    if not build_type:
        raise ValueError(f"{package_xml} is missing <export><build_type>")
    return build_type


def _description(package_xml: Path) -> str:
    tree = ET.parse(package_xml)
    return (tree.findtext("description") or "").strip()


def _records() -> list[PackageRecord]:
    records: list[PackageRecord] = []
    for package_xml in sorted(SRC.glob("*/package.xml")):
        records.append(
            PackageRecord(
                name=_package_name(package_xml),
                path=package_xml.parent,
                build_type=_build_type(package_xml),
                description=_description(package_xml),
            )
        )
    return records


def _validate(records: list[PackageRecord]) -> list[str]:
    errors: list[str] = []
    names = [record.name for record in records]
    if len(names) != len(set(names)):
        errors.append("duplicate package names in ros2_ws/src")

    for record in records:
        relative = record.path.relative_to(ROOT)
        if record.name != record.path.name:
            errors.append(f"{relative}: package name {record.name!r} does not match directory")
        if record.build_type == "ament_python":
            if not (record.path / "setup.py").exists():
                errors.append(f"{relative}: ament_python package is missing setup.py")
            if not (record.path / "setup.cfg").exists():
                errors.append(f"{relative}: ament_python package is missing setup.cfg")
            if not (record.path / "resource" / record.name).exists():
                errors.append(f"{relative}: missing resource/{record.name}")
            if not (record.path / record.name / "__init__.py").exists():
                errors.append(f"{relative}: missing {record.name}/__init__.py")
        elif record.build_type == "ament_cmake":
            if not (record.path / "CMakeLists.txt").exists():
                errors.append(f"{relative}: ament_cmake package is missing CMakeLists.txt")
        else:
            errors.append(f"{relative}: unsupported build_type {record.build_type!r}")
    return errors


def main() -> int:
    records = _records()
    errors = _validate(records)

    print("Package inventory")
    print("=================")
    for record in records:
        print(f"{record.name:<22} {record.build_type:<13} {record.description}")

    if errors:
        print("\nInventory errors", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

