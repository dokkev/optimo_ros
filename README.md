# Optimo ROS

Collection of ROS2 packages for the Optimo Robot.
<!-- Put these in alphabetical order. Thats how they show up on the repository source page. -->
- `optimo_api`: ROS2 wrapper for interfacing with the Optimo-specific Roboligent SDK libraries.
- `optimo_bringup`: ROS2 launch and config files to bring up the Optimo Robot.
- `optimo_controllers`: ROS2 controllers for the Optimo Robot.
- `optimo_description`: URDF of the Optimo Robot.
- `optimo_hardware`: ROS2 wrapper for interfacing with the Optimo Robot's hardware.
- `optimo_moveit_config`: MoveIt configuration for the Optimo Robot.
- `optimo_msgs`: ROS2 services, messages and actions for the Optimo Robot.
- `optimo_quick_start_app`: Debug GUI application for the Optimo Robot.
- `optimo_ros`: ROS2 metapackage for the all the optimo packages.
- `optimo_sim`: For running the Optimo in Gazebo.


## Turn on the robot (only applied to AHG Optimo)
On Arduino
<r,s,0> - Turn on the robot
<r,s,1> - Turn off the robot


### Launching

To view and play around with the model (URDF) in Rviz, run the following:

```bash
ros2 launch optimo_description rviz_rsp.launch.py
```

To bringup the robot in totality, run the following:

```bash
ros2 launch optimo_bringup optimo.launch.py
```

- `ns`: Changes the namespace (default: `optimo`)
- `use_sim_hardware`: Set to `True` for Gazebo simulation, `False` for real hardware (default: `False`)
- `robot_index`: Specifies the ethernet port the robot is connected to (default: `0`)
- `use_gripper`: Set to `True` if using a gripper (default: `False`)

### Services

All services must be prefixed with the namespace, which is set to `optimo` by default. These examples will use the default namespace, but keep in mind that this can be changed, especially when running multiple robots.

#### Enabling the Robot

The robot is always enabled in Gazebo and enables after configuring when running with real hardware. To manually enable the robot with real hardware, use the service call:

```bash
# This service only works with real hardware.
ros2 service call /optimo/optimo_hw_iface_server/toggle_enable std_srvs/srv/SetBool "{data: true}"
```

To disable the robot, set data to false.


## Teleoperation

### Start Teleop

launch teleop code to start teleop mode
```
ros2 launch optimo_teleop optimo_teleop.launch.py
```

### To use spacemouse
```
ros2 launch optimo_teleop spacemouse_teleop.launch.py 

```

### To end teleop

```
ros2 service call /optimo/optimo_effort_controller/stop_cb std_srvs/srv/Trigger


```

### To enable robot when it faults
```
ros2 service call /optimo/optimo_hw_iface_server/toggle_enable std_srvs/srv/SetBool "{data: true}"
```
### Disabling the breaks when hits joint limits
```
cd ~/CODE/roboligent_sdk/optimo_controller
./bin/quick_start
```
Start hardware and let it load
Disable brakes (WARNING: ROBOT WILL FALL IF NOT SUPPORTED)
Turn off harware

## PLATO Hand

### To launch Plato hand (this starts contact estimation)
```
ros2 launch plato2_hardware_interface plato2_finger_hardware.launch.py 

```

Contact info can be obtained from the following topic:
```
/plato2/b_contact_index
```



#### Commanding Callbacks

When no callback is commanded to the Optimo, the controller uses idle callback, meaning that it anchors its position but is still pliable and can be dragged to a new position.

To command a free motion callback:

```bash
ros2 service call /optimo/optimo_effort_controller/free_motion_cb optimo_msgs/srv/GenericCb "{duration: 3}"
```

To end any callback (other than idle):

```bash
ros2 service call /optimo/optimo_effort_controller/stop_cb std_srvs/srv/Trigger 
```

To move the robot back to the home position:

```bash
ros2 service call /optimo/optimo_effort_controller/move_home_cb optimo_msgs/srv/PosCb
```

To move to a custom position:

```bash
ros2 service call /optimo/optimo_effort_controller/move_home_cb optimo_msgs/srv/PosCb "{pos: [0,3.3,0,-2.35,0,-1.13,0]}"
```

#### Teaching and Playing Trajectories

To teach a trajectory:

```bash
ros2 service call /optimo/optimo_effort_controller/teach_traj_cb optimo_msgs/srv/StringCb "{string: '<add absolute filepath here>'}"
```

To play a trajectory:

```bash
ros2 service call /optimo/optimo_effort_controller/play_traj_cb optimo_msgs/srv/PlayTrajCb "{goal: {filepath: '<add absolute filepath here>', param: {imp: {impedance: 8, max_force: 2, stop_sensitivity: 2}, recovery_speed: 5, accept_diff_start: true}}}"
```

Parameters for trajectory execution:
- Impedance: Ratio of force applied to position offset (1-10).
- Max force: Maximum force applied to follow the trajectory (1-10).
- Stop Sensitivity: Distance offset before execution pauses (1-10, lower is stricter).
- Recovery speed: Speed of returning to the original trajectory after pausing (1-10).
- accept_diff_start: If true, the robot moves to the starting position before starting the trajectory.

All parameters are set to default values in the example service call and when omitted.

To calculate the end effector load, you can call the following service:

```bash
ros2 service call /dex/left_arm/optimo_effort_controller/calculate_load std_srvs/srv/Trigger
```

This will enable the load calculation for the left arm, and wait until the load calculation is finished.

## Licensing

This project is licensed under the [Apache 2.0 license][apache-2.0].

## Copyright

Copyright 2024 Roboligent, Inc.


[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[Roboligent-Documentation]: https://www.roboligent.bitbucket.io
