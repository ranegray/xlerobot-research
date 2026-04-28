from __future__ import annotations

from pathlib import Path

from scripts.validate_run_card import load_run_card, validate_run_card


EXAMPLE = Path(__file__).parents[1] / "examples" / "run_cards" / "fake_one_arm_smoke.yaml"


def test_example_run_card_is_valid() -> None:
    errors = validate_run_card(load_run_card(EXAMPLE))

    assert errors == []


def test_run_card_rejects_missing_required_fields() -> None:
    errors = validate_run_card({"schema": "xle.run_card.v0"})

    assert "missing required field: run_id" in errors
    assert "evidence must be a mapping" in errors


def test_run_card_rejects_fake_real_mode_typos() -> None:
    errors = validate_run_card(
        {
            "schema": "xle.run_card.v0",
            "run_id": "bad",
            "repo_commit": "abc123",
            "hardware_manifest": {"mode": "sim"},
            "calibration_bundle": {},
            "task": "test",
            "start_state": "start",
            "target_state": "target",
            "operator_reset": "reset",
            "launch_command": "ros2 launch ...",
            "bag_path": "bags/example",
            "success": False,
            "failure_tag": "bad_mode",
            "manual_interventions": [],
            "notes": "",
            "evidence": {
                "topics": [],
                "rosbag_info": "",
                "checksums": [],
                "artifact_links": [],
            },
        }
    )

    assert "hardware_manifest.mode must be 'fake' or 'real'" in errors
