# msgs_pkg

ROS 2 interfaces shared by perception and manipulation packages.

## Contents

### Messages

- `Detection.msg`
  - Single detected object with class name/id, confidence, and 3D position.
- `DetectionArray.msg`
  - Header + array of `Detection`.
- `MotionStep.msg`
  - One step for sequence execution (`go_to_pose`, `go_to_named`, `set_gripper`).

### Actions

- `PickPlace.action`
  - Pick from one position and place at another.
- `MotionSequence.action`
  - Execute a list of `MotionStep` entries.

## Build

```bash
cd /home/pavan/p_ws
colcon build --symlink-install --packages-select msgs_pkg
source install/setup.bash
```

## Verify

```bash
ros2 interface list | grep msgs_pkg
ros2 interface show msgs_pkg/msg/Detection
ros2 interface show msgs_pkg/msg/DetectionArray
ros2 interface show msgs_pkg/msg/MotionStep
ros2 interface show msgs_pkg/action/PickPlace
ros2 interface show msgs_pkg/action/MotionSequence
```

## Dependencies

- Build: `ament_cmake`, `rosidl_default_generators`
- Runtime: `rosidl_default_runtime`, `geometry_msgs`, `std_msgs`

## Notes

- Rebuild this package whenever `.msg` or `.action` files change.
- Downstream packages (`segmenta`, `move_7sk`) depend on these interfaces.
