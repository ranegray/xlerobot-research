PYTHON ?= python3

.PHONY: setup lint test build smoke ci-local

setup:
	cd ros2_ws && rosdep install --from-paths src --ignore-src -r -y

lint:
	$(PYTHON) -m compileall -q ros2_ws/src tests scripts
	bash -n scripts/fake_launch_smoke.sh
	$(PYTHON) scripts/check_package_inventory.py
	$(PYTHON) scripts/check_launch_contracts.py
	$(PYTHON) scripts/validate_run_card.py examples/run_cards/fake_one_arm_smoke.yaml

test:
	$(PYTHON) -m pytest

build:
	cd ros2_ws && colcon build --symlink-install

smoke:
	scripts/fake_launch_smoke.sh

ci-local: lint test
