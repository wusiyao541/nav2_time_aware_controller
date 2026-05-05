# Nav2 Time-Aware Controller

## Overview

This ROS 2 Humble package provides a custom Nav2 controller plugin for time-aware waypoint navigation. It tracks the path supplied by Nav2 while adjusting speed so each `NavigateToPose` segment can be driven toward a requested travel time.

The controller plugin class is:

```text
nav2_time_aware_controller::TimeAwareLQRController
```

It is exported through `plugins.xml` as a `nav2_core::Controller`.

## What the Plugin Does

The plugin compares remaining path distance with remaining requested time and uses that timing error to shape the linear reference speed. Longer path segments can be driven faster, shorter segments can be driven slower, and late segments can become more aggressive within the configured speed and acceleration limits.

Near the goal, goal-approach logic takes priority so the robot slows down and arrives stably.

## Controller Design

The controller uses a Frenet-frame path representation with finite-horizon LQR tracking:

1. Nav2 provides a global path to the controller.
2. The controller computes path arc length, tangent heading, and curvature.
3. A time-indexed reference trajectory is generated from `desired_segment_time`.
4. The robot pose is projected onto the path.
5. Longitudinal, lateral, and heading errors are computed in the path frame.
6. LQR feedback adjusts linear and angular velocity.
7. Time-aware logic increases reference speed when the robot is behind schedule.
8. Velocity, acceleration, angular, and goal-approach limits clamp the command.

Important files:

- `include/nav2_time_aware_controller/time_aware_lqr_controller.hpp`
- `src/lifecycle.cpp`
- `src/control_loop.cpp`
- `src/time_logic.cpp`
- `src/path_utils.cpp`
- `src/lqr_math.cpp`
- `src/safety_limits.cpp`
- `src/debug.cpp`

## Build

From the workspace root:

```bash
cd ~/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_time_aware_controller
source install/setup.bash
```

## Nav2 Configuration

Use the plugin under `controller_server`:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_time_aware_controller::TimeAwareLQRController"
      desired_segment_time: 20.0
```

`desired_segment_time` is the requested travel time for the current Nav2 path segment / current `NavigateToPose` goal. It is not the total patrol-loop time.

The example configuration is in `config/nav2.yaml`.

## Key Parameters

Core timing and speed parameters:

```yaml
desired_segment_time: 20.0
max_v: 0.38
min_tracking_v: 0.06
fastest_tracking_v: 0.38
time_aggressiveness: 1.15
max_accel: 0.60
max_decel: 0.80
max_w: 1.0
max_angular_accel: 1.8
command_filter_alpha: 0.9
```

Goal approach parameters:

```yaml
goal_tolerance: 0.18
goal_slowdown_dist: 0.30
goal_approach_dist: 0.34
goal_approach_gain: 0.55
min_goal_approach_v: 0.08
max_goal_approach_v: 0.14
max_goal_approach_w: 0.4
near_path_end_dist: 0.22
```

LQR parameters:

```yaml
q_s: 1.0
q_y: 12.0
q_psi: 8.0
r_v: 3.0
r_w: 2.5
qf_s: 2.0
qf_y: 16.0
qf_psi: 10.0
lqr_horizon_steps: 20
max_delta_v: 0.12
max_delta_w: 0.8
```

## Time-Aware Speed Logic

For each control cycle, the controller estimates:

```text
required_average_speed = remaining_path_distance / remaining_segment_time
```

If the robot is behind the expected path progress, it applies:

```text
aggressive_speed = required_average_speed * time_aggressiveness
```

The result is clamped by `max_v`, acceleration limits, command filtering, and goal-approach constraints. If the requested timing is physically infeasible, the controller uses `fastest_tracking_v` within the same safety limits.

## Goal Approach Behavior

Near the final path endpoint, the controller switches into goal-approach mode. This mode limits forward speed, checks heading error, and prevents unstable arrival behavior around the waypoint.

Tune `goal_tolerance`, `goal_slowdown_dist`, `goal_approach_dist`, `min_goal_approach_v`, and `max_goal_approach_v` together with the Nav2 goal checker tolerance.

## Runtime Diagnostics

Set `debug: true` in `config/nav2.yaml` to enable throttled controller logs. The main diagnostic line reports distance remaining, time remaining, required speed, schedule error, command limiting stages, controller mode, and limiting reason.

Useful fields include:

- `path_rem` - remaining path distance.
- `time_left` - remaining requested segment time.
- `req_v` - required average speed.
- `aggr_v` - aggressive tracking speed after clamping.
- `sched_err` - progress error relative to the requested timing.
- `final_v` - final linear velocity command.
- `mode` - active control mode.
- `reason` - main reason the command was limited.

CSV logging can be enabled with `debug_csv_path`.

## Integration Notes

The package includes `timed_waypoint_runner`, a helper node that sends `NavigateToPose` goals and updates `FollowPath.desired_segment_time` before each goal when a waypoint has `target_time` set.

Waypoint timing is configured in `config/timed_waypoints.yaml`:

```yaml
waypoints:
  names: ["goal_1"]
  goal_1:
    x: -3.0
    y: 1.0
    yaw: 3.14
    target_time: 20.0
```

The plugin itself does not own the waypoint sequence, publish goals, or cancel goals. It only computes velocity commands for the path supplied by Nav2.

## Current Status

The controller is currently tuned for TurtleBot4 simulation and produces stable segment-level time-aware patrol behavior. Retune speed limits, LQR weights, and goal-approach parameters for different robots, maps, or waypoint spacing.
