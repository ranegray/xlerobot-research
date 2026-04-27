# Fake Hardware

Fake hardware is the first simulation layer. It exists to prove that launch files, topic names, command messages, TF, and bags work before any servo can move.

## Launch

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch xle_bringup fake_one_arm.launch.py
```

Expected first topics:

- `/joint_states`
- `/tf`
- `/tf_static`
- `/harness/events`
- `/left_arm_controller/joint_trajectory`
- `/left_arm_controller/guarded_joint_trajectory`
- `/right_arm_controller/joint_trajectory`
- `/right_arm_controller/guarded_joint_trajectory`

## Send A Small Fake Command

```bash
ros2 topic pub --once /left_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['Rotation_L', 'Pitch_L', 'Elbow_L', 'Wrist_Pitch_L', 'Wrist_Roll_L'], points: [{positions: [0.05, 0.10, 0.10, 0.00, 0.00], time_from_start: {sec: 2}}]}"
```

The fake STS3215 node clamps commands to the imported URDF joint limits and moves simulated joint positions toward the target at a bounded rate.

The guard publishes structured JSON events on `/harness/events` when it accepts or rejects a command.

## Record The First Bag

```bash
ros2 bag record /joint_states /tf /tf_static /harness/events /left_arm_controller/joint_trajectory /left_arm_controller/guarded_joint_trajectory /right_arm_controller/joint_trajectory /right_arm_controller/guarded_joint_trajectory
```

This bag is not a manipulation result. It is evidence that command, state, and frame data can be observed together.
