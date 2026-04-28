# Launch Inventory

Use this as the quick map before editing launch files.

| launch file | purpose | hardware risk | CI use |
| --- | --- | --- | --- |
| `xle_bringup/launch/fake_one_arm.launch.py` | fake one-arm runtime with robot state publisher, guard, and fake STS3215 node | none | yes, via `make smoke` |
| `xle_bringup/launch/view_fake_arm.launch.py` | fake one-arm runtime plus RViz visualization | none, but GUI required | no |
| `xle_bringup/launch/real_one_arm_left.launch.py` | left-arm bus1 hardware bridge with guard and robot state publisher | real hardware; torque default must stay `false` | no |
| `xle_bringup/launch/view_real_arm.launch.py` | real left-arm bridge plus RViz visualization | real hardware; torque default must stay `false` | no |
| `xle_description/launch/view_robot.launch.py` | URDF robot state publisher only | none | optional |
| `xle_perception/launch/realsense.launch.py` | RealSense launch plus camera-link static transform | camera required | no |

## Static Checks

Run:

```bash
python3 scripts/check_launch_contracts.py
```

The checker verifies:

- launch-referenced internal console scripts exist
- launch-referenced packages are declared as runtime dependencies
- included local launch files exist
- `enable_torque` defaults stay `false`

It does not replace a real ROS launch. It catches low-level drift before a branch reaches ROS CI.

