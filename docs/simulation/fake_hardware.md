# Fake Hardware

Fake hardware is the first simulation layer. It exists to prove that launch files, topic names, command messages, TF, and bags work before any servo can move.

## Build And Launch

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

## Observe

In separate terminals, source the workspace and inspect the fake runtime:

```bash
cd ros2_ws
source install/setup.bash
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic echo /harness/events --once
```

`/joint_states` should include the left arm, right arm, wheels, grippers, and head joints. `/harness/events` emits `xle.harness.event.v0` JSON strings from the guard when a command is accepted or rejected.

## Send A Small Fake Command

```bash
ros2 topic pub --once /left_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['Rotation_L', 'Pitch_L', 'Elbow_L', 'Wrist_Pitch_L', 'Wrist_Roll_L'], points: [{positions: [0.05, 0.10, 0.10, 0.00, 0.00], time_from_start: {sec: 2}}]}"
```

The same command is tracked as a test fixture:

```bash
python3 scripts/publish_trajectory_fixture.py --print tests/fixtures/trajectories/valid_left_arm_small.yaml
python3 scripts/publish_trajectory_fixture.py tests/fixtures/trajectories/valid_left_arm_small.yaml
```

The fake STS3215 node clamps commands to the imported URDF joint limits and moves simulated joint positions toward the target at a bounded rate.

The guard publishes structured JSON events on `/harness/events` when it accepts or rejects a command.

To test the rejection path, send a right-arm joint to the left-arm controller:

```bash
ros2 topic pub --once /left_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['Rotation_R'], points: [{positions: [0.0], time_from_start: {sec: 1}}]}"
```

The guard should reject the command and publish a `command_rejected` event.

Fixture equivalents:

```bash
python3 scripts/publish_trajectory_fixture.py tests/fixtures/trajectories/invalid_left_arm_right_joint.yaml
python3 scripts/publish_trajectory_fixture.py tests/fixtures/trajectories/invalid_left_arm_out_of_limit.yaml
```

## Record The First Bag

```bash
mkdir -p ~/rosbags/xlerobot-research
ros2 bag record \
  -o ~/rosbags/xlerobot-research/fake_one_arm_$(date +%Y%m%d_%H%M%S) \
  /joint_states \
  /tf \
  /tf_static \
  /harness/events \
  /left_arm_controller/joint_trajectory \
  /left_arm_controller/guarded_joint_trajectory \
  /right_arm_controller/joint_trajectory \
  /right_arm_controller/guarded_joint_trajectory
```

This bag is not a manipulation result. It is evidence that command, state, and frame data can be observed together.

After recording:

```bash
ros2 bag info ~/rosbags/xlerobot-research/<bag-directory>
```

Commit run cards, checksums, and links to artifacts. Do not commit `.db3`, `.mcap`, or large bag files to normal git history.

Use the fake smoke run-card example as the starting evidence record:

```bash
cp examples/run_cards/fake_one_arm_smoke.yaml /tmp/fake_one_arm_run.yaml
python3 scripts/validate_run_card.py /tmp/fake_one_arm_run.yaml
```

## Smoke Test

After `make build`, run:

```bash
make smoke
```

The smoke script launches `xle_bringup fake_one_arm.launch.py`, waits for the guard and fake hardware nodes to report ready, observes `/joint_states`, publishes `tests/fixtures/trajectories/valid_left_arm_small.yaml`, and requires a `command_accepted` event on `/harness/events`.
