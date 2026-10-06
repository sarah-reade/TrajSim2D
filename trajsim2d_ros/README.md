# TrajSim2D ROS 2 wrapper

This package exposes the core simulator as the `trajsim2d` node.

## Interfaces

- Subscribes to `trajectory_msgs/msg/JointTrajectory` on
  `/motion_planner/trajectory`. Each point must contain positions for all
  joints, optional velocities for all joints, and strictly increasing
  `time_from_start` values. Joint names must be `joint_1`, `joint_2`, etc.
- Publishes `sensor_msgs/msg/JointState` on `/robot_state`.
- Publishes `geometry_msgs/msg/PoseStamped` on `/goal_pose`. The pose is the
  end-effector pose in `map`.
- Publishes `nav_msgs/msg/OccupancyGrid` on `/cost_map`.
- Publishes `tf2_msgs/msg/TFMessage` on `/tf`, containing `map` to
  `base_link` and each `link_N` transform.
- Provides the `trajsim2d_ros/action/Simulation` action on `/simulation`.

The action goal values are `0` (get current state), `1` (generate random
environment), `2` (load environment from file), `3` (set the trajectory
processing save location), and `4` (save environment). The action feedback
reports `VACANT`, `STARTUP`, `IDLE`, or `SIMULATING`.

## Running

Build this package in a ROS 2 workspace after installing the Python core
package:

```bash
colcon build --packages-select trajsim2d_ros --symlink-install
ros2 run trajsim2d_ros trajsim2d_node
```

An environment is loaded through the action using a `.canvas` file produced
by `save_canvas_to_file`. The canvas contains the relative path to its
manipulator URDF; loading the canvas reconstructs both the manipulator and
the scene, so no separate URDF parameter or input is used. Before an
environment is loaded the node reports `VACANT`.
