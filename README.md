# Nav2 Time-Aware Controller

This package implements a custom ROS2 Nav2 controller plugin for time-aware navigation.

## Features

- Custom Nav2 controller plugin
- Time-aware reference velocity generation
- LQR-based path tracking
- Infeasible-time feedback
- Modular C++ implementation

## Build

```bash
cd ~/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_time_aware_controller
source install/setup.bash