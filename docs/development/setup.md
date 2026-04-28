# Developer Setup

Target environment for the full ROS path:

- Ubuntu 22.04
- ROS 2 Humble
- `python3-colcon-common-extensions`
- `rosdep`

From a fresh checkout:

```bash
cd xlerobot-research
make setup
make build
```

If this is the first ROS workspace on the machine and `rosdep` is not initialized, run the standard one-time ROS setup first:

```bash
sudo rosdep init
rosdep update
```

For docs, tests, schemas, and analysis scripts, ROS is not required:

```bash
python3 -m pip install pytest PyYAML
make ci-local
```

The local Python checks intentionally cover code that can run without `rclpy`. Full launch and package checks still require ROS 2 Humble.

## Common Commands

```bash
make lint       # compile Python and validate package inventory
make test       # run local unit tests
make build      # colcon build in ros2_ws
make smoke      # fake one-arm launch smoke after build
```

See `docs/development/launch_inventory.md` before changing launch files.

## Fake Runtime

The fake runtime is the default first run for contributors:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch xle_bringup fake_one_arm.launch.py
```

See `docs/simulation/fake_hardware.md` for command, observation, and recording steps.
