# mover

Python helper package that calls `moveit_7sk` services and runs simple pick/place sequences.

## Modules

- `mover/client.py`
  - `MoveBotClient` wraps service clients:
    - `go_to_named_pose`
    - `go_to_pose`
    - `set_gripper`
- `mover/sequence.py`
  - `SequenceRunner` executes action lists with `pick` and `place` steps.
- `mover/test.py`
  - Example executable sequence (`test_sequence` console script).

## Package Dependency

This package requires `moveit_7sk` services to be running.

## Build

```bash
cd /home/pavan/p_ws
colcon build --symlink-install --packages-select moveit_7sk mover
source install/setup.bash
```

## Run Example Sequence

Start Gazebo and spawn cubes first:

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"
ros2 run realsense_camera_pkg spawn_cubes
```

Then start MoveIt API:

```bash
ros2 run moveit_7sk moveit_7sk_node
```

Then run the test sequence:

```bash
ros2 run mover test_sequence
```

## Sequence Input Format

`SequenceRunner.execute()` expects:

```json
{
  "actions": [
    {
      "type": "pick",
      "x": 0.2,
      "y": 0.3,
      "z": 0.03,
      "gripper_val": -0.02
    },
    {
      "type": "place",
      "x": -0.2,
      "y": 0.3,
      "z": 0.03,
      "gripper_val": 0.02
    }
  ]
}
```

Optional per-action fields:

- `roll` (default: `-1.57`)
- `pitch` (default: `0.0`)
- `yaw` (default: `0.0`)
- `approach_height` (default: `0.15`)
- For `pick`, `grasp_height` (default: `0.1`)

## Troubleshooting

- Service wait hangs: ensure `moveit_7sk_node` is running and service names match.
- Motion fails: verify planning scene, reachable targets, and MoveIt controller setup.
