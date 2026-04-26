# XLeRobot URDF Provenance

Imported on: 2026-04-26

Upstream project: [Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot)

Local upstream checkout: `/Users/ranegray/Development/robotics/XLeRobot`

Upstream commit used: `137865981ca9e828d0923804cf77ededd22c7816`

Original archive: `simulation/xlerobot_urdf.zip`

Imported files:

- `ros2_ws/src/xle_description/urdf/xlerobot_0_4_0_vendor.urdf`
- `ros2_ws/src/xle_description/urdf/xlerobot.urdf`
- `ros2_ws/src/xle_description/meshes/xlerobot/`

Import changes:

- Rewrote URDF mesh references from `meshes/xlerobot/assets/...` to `package://xle_description/meshes/xlerobot/assets/...` in both URDF files.
- Preserved `xlerobot_0_4_0_vendor.urdf` as the vendor URDF with no intentional joint edits.
- Made `xlerobot.urdf` the default parked-base ROS derivative by changing root `joint_0` from `floating` to `fixed`.
- Did not edit geometry, arm joints, inertial properties, collision meshes, limits, or frame names.

License handling:

- The upstream XLeRobot repository is licensed under Apache-2.0 at the checked commit.
- A local copy of the upstream license is stored at `third_party/XLeRobot-LICENSE`.

Verification notes:

- This import is ready for ROS package resolution.
- Runtime verification still requires a ROS 2 environment with `robot_state_publisher` and mesh loading tools available.
