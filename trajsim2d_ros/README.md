# TrajSim2D ROS 2 wrapper

This package exposes the core simulator as the `trajsim2d` node.

## Interfaces

- Subscribes to `trajsim2d_ros/msg/JointVelocity` on
  `/motion_planner/trajectory`. The message contains `name` and `velocity`
  arrays, with joint names `joint_1`, `joint_2`, etc.
- Publishes `sensor_msgs/msg/JointState` on `/robot_state`.
- Publishes `geometry_msgs/msg/PoseStamped` on `/goal_pose`. The pose is the
  end-effector pose in `map`.
- Publishes `nav_msgs/msg/OccupancyGrid` on `/cost_map`.
- Publishes `tf2_msgs/msg/TFMessage` on `/tf`, containing `map` to
  `base_link` and each `link_N` transform.
- Provides the `trajsim2d_ros/action/Simulation` action on `/simulation`.

The action goal values are `0` (set save location), `1` (load environment),
`2` (save environment), and `3` (request current state). The action feedback
reports `VACANT`, `STARTUP`, `IDLE`, or `SIMULATING`.

## Running

Build this package in a ROS 2 workspace after installing the Python core
package:

```bash
colcon build --packages-select trajsim2d_ros
ros2 run trajsim2d_ros trajsim2d_node --ros-args -p urdf:=/path/to/arm.urdf
```

An environment is loaded through the action using a `.canvas` file produced
by `save_canvas_to_file`. Before an environment is loaded the node reports
`VACANT`.
