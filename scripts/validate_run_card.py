#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import yaml


SCHEMA = "xle.run_card.v0"
REQUIRED_FIELDS = [
    "schema",
    "run_id",
    "repo_commit",
    "hardware_manifest",
    "calibration_bundle",
    "task",
    "start_state",
    "target_state",
    "operator_reset",
    "launch_command",
    "bag_path",
    "success",
    "failure_tag",
    "manual_interventions",
    "notes",
    "evidence",
]
EVIDENCE_REQUIRED_FIELDS = [
    "topics",
    "rosbag_info",
    "checksums",
    "artifact_links",
]


def load_run_card(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def validate_run_card(data: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    for field in REQUIRED_FIELDS:
        if field not in data:
            errors.append(f"missing required field: {field}")

    if data.get("schema") != SCHEMA:
        errors.append(f"schema must be {SCHEMA!r}")

    if "success" in data and not isinstance(data["success"], bool):
        errors.append("success must be a boolean")

    if "manual_interventions" in data and not isinstance(data["manual_interventions"], list):
        errors.append("manual_interventions must be a list")

    evidence = data.get("evidence")
    if not isinstance(evidence, dict):
        errors.append("evidence must be a mapping")
    else:
        for field in EVIDENCE_REQUIRED_FIELDS:
            if field not in evidence:
                errors.append(f"evidence missing required field: {field}")
        if "topics" in evidence and not isinstance(evidence["topics"], list):
            errors.append("evidence.topics must be a list")
        if "artifact_links" in evidence and not isinstance(evidence["artifact_links"], list):
            errors.append("evidence.artifact_links must be a list")

    hardware = data.get("hardware_manifest")
    if isinstance(hardware, dict):
        mode = hardware.get("mode")
        if mode not in {"fake", "real"}:
            errors.append("hardware_manifest.mode must be 'fake' or 'real'")
    elif "hardware_manifest" in data:
        errors.append("hardware_manifest must be a mapping")

    return errors


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate xle.run_card.v0 YAML files.")
    parser.add_argument("run_cards", nargs="+", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    failed = False
    for path in args.run_cards:
        errors = validate_run_card(load_run_card(path))
        if errors:
            failed = True
            print(f"{path}: invalid", file=sys.stderr)
            for error in errors:
                print(f"- {error}", file=sys.stderr)
        else:
            print(f"{path}: ok")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

