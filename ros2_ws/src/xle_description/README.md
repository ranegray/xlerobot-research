# xle_description

`xle_description` contains the vendored XLeRobot 0.4.0 URDF and mesh assets used by this stack.

The model was imported from the upstream XLeRobot repository archive at `simulation/xlerobot_urdf.zip`. Mesh references in the copied URDF were rewritten from relative archive paths to ROS package URIs so `robot_state_publisher`, RViz, and downstream launch files can resolve them after installation.

URDF files:

- `urdf/xlerobot_0_4_0_vendor.urdf`: vendor import with ROS package mesh URIs and no intentional joint edits.
- `urdf/xlerobot.urdf`: default parked-base ROS model. This keeps the vendor geometry but changes the root `joint_0` from `floating` to `fixed` so the first fake-hardware launch can publish a useful static `world -> chassis` transform before mobile-base state exists.

Provenance is tracked in [`third_party/XLeRobot-URDF-PROVENANCE.md`](../../../third_party/XLeRobot-URDF-PROVENANCE.md). The upstream license copy is stored at [`third_party/XLeRobot-LICENSE`](../../../third_party/XLeRobot-LICENSE).

## First Launch Target

```bash
cd ros2_ws
colcon build --symlink-install --packages-select xle_description
source install/setup.bash
ros2 launch xle_description view_robot.launch.py
```

This should publish `robot_description` and start `robot_state_publisher`. Fake or real joint state publication is handled by later packages.
