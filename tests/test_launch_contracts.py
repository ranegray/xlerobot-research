from __future__ import annotations

from scripts.check_launch_contracts import (
    SRC,
    launch_contract,
    package_dirs,
    validate_contract,
)


def test_all_launch_files_match_static_contracts() -> None:
    packages = package_dirs()
    errors = []
    for path in sorted(SRC.glob("*/launch/*.py")):
        errors.extend(validate_contract(launch_contract(path), packages))

    assert errors == []


def test_real_launch_keeps_torque_disabled_by_default() -> None:
    contract = launch_contract(
        SRC / "xle_bringup" / "launch" / "real_one_arm_left.launch.py"
    )

    assert [default for default, _line in contract.enable_torque_defaults] == ["false"]
