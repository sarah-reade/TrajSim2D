# TrajSim2D ROS 2 wrapper

This package exposes the core simulator as the `trajsim2d` node.

## Interfaces

- Subscribes to `trajectory_msgs/msg/JointTrajectory` on
  `/motion_planner/trajectory`. Each point must contain positions for all
  joints, optional velocities for all joints, and strictly increasing
  `time_from_start` values. Joint names must be `joint_1`, `joint_2`, etc.
- Publishes `sensor_msgs/msg/JointState` on `/robot_state`.
- Publishes `sensor_msgs/msg/JointState` on `/goal_pose`. The message contains
  the goal joint configuration.
- Publishes `nav_msgs/msg/OccupancyGrid` on `/cost_map`.
- Publishes the current manipulator URDF as `std_msgs/msg/String` on
  `/robot_description` whenever an environment is loaded or generated. This
  topic uses transient-local QoS so late subscribers receive the latest
  description. It is not published while the node is `VACANT`.
- Publishes `tf2_msgs/msg/TFMessage` on `/tf`, containing `map` to
  `base_link` and each `link_N` transform.
- Provides the `trajsim2d_ros/action/Simulation` action on `/simulation`.
- The `headless` parameter defaults to `false`. Set it to `true` to disable
  the Matplotlib visualisation.

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

### One-shot direct trajectory test

Start the simulator first so that `/robot_state` and `/goal_pose` are
available:

```bash
ros2 run trajsim2d_ros trajsim2d_node --ros-args -p headless:=true
```

Then start the test node:

```bash
ros2 run trajsim2d_ros direct_trajectory_test_node
```

The test node waits for the first messages on `/robot_state` and `/goal_pose`,
then publishes exactly one `trajectory_msgs/msg/JointTrajectory` message on
`/motion_planner/trajectory`. The trajectory is a direct straight line in
joint configuration space with a smoothstep time profile and zero velocity at
both endpoints. It contains 21 points and takes three seconds by default.

Change the duration or number of points with ROS parameters:

```bash
ros2 run trajsim2d_ros direct_trajectory_test_node --ros-args \
  -p duration:=5.0 \
  -p samples:=31
```

After publishing, the test node stays alive but ignores subsequent state
updates, so it can be stopped with `Ctrl-C`.

To run the simulator with its visualisation enabled instead, omit
`-p headless:=true` from the simulator command.

An environment is loaded through the action using a `.canvas` file produced
by `save_canvas_to_file`. The canvas contains the relative path to its
manipulator URDF; loading the canvas reconstructs both the manipulator and
the scene, so no separate URDF parameter or input is used. Before an
environment is loaded the node reports `VACANT`.
