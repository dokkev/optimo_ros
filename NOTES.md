# Optimo ROS Package Notes

## optimo_bringup

The main orchestrator package that brings up the entire Optimo 7-DOF robot arm system. It launches robot state publishing, hardware communication (real EtherCAT or Gazebo sim), and the ros2_control stack.

### Nodes Launched

| Node | Purpose |
|------|---------|
| robot_state_publisher | Publishes URDF → TF tree |
| ros2_control_node | Controller manager (real hardware) |
| joint_state_broadcaster | Broadcasts joint states at 50 Hz |
| optimo_effort_controller | Main torque-based arm controller |
| gripper_position_controller | Gripper position control (optional) |
| gripper_effort_controller | Gripper effort control (optional) |
| Gazebo + spawner | Simulation only (when `use_sim_hardware=True`) |

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `ns` | `"optimo"` | Namespace for all nodes |
| `use_sim_hardware` | `"False"` | Real EtherCAT vs Gazebo sim |
| `robot_index` | `"0"` | EtherCAT master index |
| `use_gripper` | `"False"` | Enable gripper controllers |
| `prefix` | `""` | Joint/link name prefix (e.g. `"left_arm_"`) |
| `controller_yaml_path` | `""` | Custom controller config path |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Joint positions, velocities, efforts |
| `/optimo/external_wrench` | `geometry_msgs/WrenchStamped` | External force at end-effector |
| `/optimo/ee_pose_current` | `optimo_msgs/PoseElbow` | Current EE pose (initialized but may be commented out) |
| `/optimo/jacobian` | `std_msgs/Float64MultiArray` | 7x6 Jacobian matrix (initialized but may be commented out) |

### Services (on `/optimo/optimo_effort_controller/`)

| Service | Type | Description |
|---------|------|-------------|
| `free_motion_cb` | `GenericCb` | Gravity compensation mode (no position control) |
| `move_home_cb` | `PosCb` | Move robot to home position |
| `joint_trajectory_cb` | `JointTrajectoryCb` | Play joint trajectory from file |
| `stop_cb` | `Trigger` | Stop all tasks, clear queue |
| `teach_traj_cb` | `StringCb` | Record/teach trajectory to file |
| `play_traj_cb` | `PlayTrajCb` | Play pre-recorded trajectory |
| `moveit_cb` | `MoveitCb` | Plan & execute via MoveIt to target pose |
| `moveit_servo_cb` | `GenericCb` | Continuous MoveIt Servo mode |
| `servo_cb` | `ServoCb` | Real-time servoing (joint or cartesian) |
| `servo_fb_cb` | `StringCb` | Servoing with force feedback |
| `calculate_load` | `Trigger` | Calculate end-effector payload |

### Hardware Interface Service

| Service | Type | Description |
|---------|------|-------------|
| `optimo_hw_iface_server/toggle_enable` | `SetBool` | Enable/disable robot hardware |

### Task Execution Model

Services queue tasks into a FIFO task queue. The controller's update loop (1000 Hz) executes tasks sequentially. No ROS 2 Actions are used — all commands go through services.

---

## optimo_teleop

Provides real-time teleoperation for the Optimo arm in two control modes: Cartesian (end-effector) and joint-space.

### Nodes

| Node | Executable | Launch File | Purpose |
|------|-----------|-------------|---------|
| optimo_teleop | teleop_node | optimo_teleop.launch.py | Cartesian/EE pose teleoperation |
| optimo_joint_teleop | joint_teleop_node | joint_teleop.launch.py | Joint-space teleoperation |
| twist_converter | twist_converter | (standalone) | Test publisher for fixed PoseElbow |

### Cartesian Mode (teleop_node)

**Subscribes:**
| Topic | Type | Description |
|-------|------|-------------|
| `/optimo/ee_pose_current` | `PoseElbow` | Current EE pose feedback |
| `/optimo/servo/twist_cmd` | `TwistStamped` | Twist velocity commands (primary input) |

**Publishes:**
| Topic | Type | Description |
|-------|------|-------------|
| `/optimo/servo/ee_pose_desired` | `PoseElbow` | Desired EE pose output |

**Parameters:** `position_scale` (default 0.01), `orientation_scale` (default 0.01)

### Joint Mode (joint_teleop_node)

**Subscribes:**
| Topic | Type | Description |
|-------|------|-------------|
| `/optimo/joint_states` | `JointState` | Current joint state feedback |
| `/optimo/joint_teleop/velocity_cmd` | `Float64MultiArray` | 7-joint velocity commands |

**Publishes:**
| Topic | Type | Description |
|-------|------|-------------|
| `/optimo/joint_teleop/desired_joint_state` | `JointState` | Desired joint positions |

**Parameters:** `velocity_scale` (default 0.01), `num_joints` (default 7)

### Control Flow

1. Launch file calls `servo_cb` service on the effort controller to configure servo mode (topic name + cartesian/joint mode)
2. 2-second startup delay for controller readiness
3. Teleop node receives velocity commands → applies scaled incremental updates → publishes desired state
4. Effort controller subscribes to the desired state topic and executes

### SpaceMouse Integration

`spacemouse_teleop.launch.py` composes nodes from `spaceMouse_navigator` and `plato_teleop` packages to enable 6-DOF SpaceMouse control.

---

## Quick Reference: Common Workflows

### Start the robot (real hardware)
```bash
ros2 launch optimo_bringup optimo.launch.py
```

### Start the robot (simulation)
```bash
ros2 launch optimo_bringup optimo.launch.py use_sim_hardware:=True
```

### Start Cartesian teleop
```bash
ros2 launch optimo_teleop optimo_teleop.launch.py
```

### Start joint teleop
```bash
ros2 launch optimo_teleop joint_teleop.launch.py
```

### Useful service calls
```bash
# Enable/disable robot
ros2 service call /optimo/optimo_hw_iface_server/toggle_enable std_srvs/srv/SetBool "{data: true}"

# Free motion (gravity comp)
ros2 service call /optimo/optimo_effort_controller/free_motion_cb optimo_msgs/srv/GenericCb "{}"

# Stop all tasks
ros2 service call /optimo/optimo_effort_controller/stop_cb std_srvs/srv/Trigger "{}"

# Move home
ros2 service call /optimo/optimo_effort_controller/move_home_cb optimo_msgs/srv/PosCb "{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
