# optimo_teleop_hardware

Hardware interfaces for Optimo robot teleoperation.

## Overview

This package provides hardware abstraction layers and ROS2 nodes for various teleoperation input devices used with the Optimo robot.

## Supported Hardware

### SpaceMouse

3D input device for intuitive 6-DOF robot control.

**Features:**
- Translation and rotation control
- Automatic mode switching based on input magnitude
- Configurable scaling and thresholds
- Button support

**Nodes:**
- `spacemouse_twist` - Publishes TwistStamped messages from SpaceMouse input

## Installation

### Dependencies

Install pyspacemouse:
```bash
pip3 install pyspacemouse
```

### Build

```bash
cd /home/optimo/CODE/ros_ws
colcon build --packages-select optimo_teleop_hardware
source install/setup.bash
```

## Usage

### SpaceMouse Teleoperation

Launch the SpaceMouse node:
```bash
ros2 launch optimo_teleop_hardware spacemouse_teleop.launch.py
```

Or run the node directly:
```bash
ros2 run optimo_teleop_hardware spacemouse_twist
```

### Parameters

The `spacemouse_twist` node accepts the following parameters:

- `translation_scale` (default: 0.01) - Scaling factor for translation commands
- `rotation_scale` (default: 0.05) - Scaling factor for rotation commands
- `translation_threshold` (default: 0.001) - Minimum magnitude for translation
- `rotation_threshold` (default: 0.001) - Minimum magnitude for rotation
- `publish_rate` (default: 900.0 Hz) - Publishing rate
- `twist_topic` (default: `/optimo/servo/twist_cmd`) - Topic for twist commands
- `frame_id` (default: `world`) - Frame ID for twist messages

### Topics

**Published:**
- `/optimo/servo/twist_cmd` (geometry_msgs/TwistStamped) - Velocity commands

## Package Structure

```
optimo_teleop_hardware/
├── optimo_teleop_hardware/
│   ├── __init__.py
│   ├── spacemouse_hardware.py      # Hardware abstraction layer
│   └── spacemouse_twist.py         # ROS2 node
├── launch/
│   └── spacemouse_teleop.launch.py
├── test/
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## License

Apache-2.0
