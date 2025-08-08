# CB SLAM Package

## Package Dependencies

- ros2 humble
- nav2
- turtlebot3
- gazebo
- slamtoolbox

## How to Use

1. Clone this package into your workspace
2. Build it with `colcon build`

## Launch Commands

To launch mapping:
```bash
ros2 launch cb_slam mapping.launch.py
```

To launch navigation:
```bash
ros2 launch cb_slam navigation.launch.py
```

To launch localization scoring and recovery node:
```bash
ros2 launch cb_slam localization_score.launch.py
```
