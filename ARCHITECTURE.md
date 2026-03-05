# Architecture: Safety-Monitored Trajectory Playback

## System Overview

The system plays pre-recorded trajectories on the Optimo 7-DOF arm while a safety monitor watches for dangerous conditions (external forces, human proximity) and stops the robot when needed.

Five components run in parallel:

```
┌──────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                          OPERATOR (CLI)                                                  │
│                                                                                                          │
│  1. ros2 service call .../record_baseline   (start baseline rec)                                         │
│  2. ros2 service call .../play_traj_cb      (play trajectory)                                            │
│  3. ros2 service call .../save_baseline     (save baseline to CSV)                                       │
│  4. (restart with baseline_file param for production use)                                                │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────┘
         │                                           │                                           │
         │ service calls                             │ service calls                             │ service calls
         ▼                                           ▼                                           ▼
┌──────────────────────┐              ┌──────────────────────────────┐              ┌──────────────────────────┐
│   OPTIMO BRINGUP     │              │    SAFETY MONITOR NODE       │    topics    │  ANTERO GRIPPER NODE     │
│                      │   topics     │                              │─────────────►│  (antero_gripper)        │
│  OptimoEffort        │─────────────►│  /optimo/external_wrench     │              │                          │
│  Controller          │              │  /optimo/joint_states        │              │  Subscribes:             │
│  (ros2_control)      │              │                              │◄─────────────│   ~/status               │
│                      │◄─────────────│  Calls: stop_cb              │    topics    │   ~/human_pose_safe      │
│  Services:           │  service     │                              │              │                          │
│   play_traj_cb       │              │  Publishes:                  │              │  Publishes:              │
│   stop_cb            │              │   ~/status (Bool)            │              │   ~/grip_status (UInt8)  │
│   free_motion_cb     │              │   ~/human_pose_safe (Bool)   │              │                          │
│   teach_traj_cb      │              │                              │              │  Action:                 │
│   move_home_cb       │              │  Safety checks:              │              │   safe → activate        │
│   moveit_cb          │              │   is_wrench_safe()           │              │   unsafe → deactivate    │
│   servo_cb           │              │   is_pose_safe()             │              │                          │
│   joint_trajectory_cb│              │                              │              │  Hardware:               │
│   calculate_load     │              │  Baseline services:          │              │   USB-to-CAN transceiver │
│                      │              │   ~/record_baseline          │              │   → ANTERO Gripper       │
│  Publishes:          │              │   ~/save_baseline            │              └──────────────────────────┘
│   /external_wrench   │              └──────────────────────────────┘
│   /joint_states      │                              ▲
│   /ee_pose_current   │                              │ /optimo/human_pose
│   /jacobian          │              ┌───────────────┴────────────────┐
│   /torque_command    │              │  HUMAN PERCEPTION NODE         │
└──────────────────────┘              │  (optimo_human_perception)     │
                                      │                                │
                                      │  Subscribes:                   │
                                      │   /camera/.../color/image_raw  │
                                      │   /camera/.../depth/image_raw  │
                                      │                                │
                                      │  Publishes:                    │
                                      │   /optimo/human_pose           │
                                      │   ~/annotated_image            │
                                      │                                │
                                      │  Uses: MediaPipe PoseLandmarker│
                                      └────────────────────────────────┘
                                                     ▲
                                              ┌──────┴───────┐
                                              │  RealSense   │
                                              │  Camera      │
                                              │  (D435/D455) │
                                              └──────────────┘
```

---

## Human Perception Node

**Package:** `optimo_human_perception`
**Executable:** `human_perception_node`
**Node name:** `human_perception` (under `/optimo` namespace)
**Language:** Python (MediaPipe Tasks API)

### ROS Interfaces

| Direction | Topic/Service | Type | Description |
|-----------|--------------|------|-------------|
| **Subscribe** | `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RGB stream from RealSense |
| **Subscribe** | `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Depth map from RealSense |
| **Publish** | `/optimo/human_pose` | `sensor_msgs/JointState` | 33 MediaPipe landmarks (interleaved x,y,z in `position` field) |
| **Publish** | `~/annotated_image` | `sensor_msgs/Image` | RGB image with skeleton overlay |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_detection_confidence` | double | 0.5 | MediaPipe pose detection confidence |
| `min_tracking_confidence` | double | 0.5 | MediaPipe tracking confidence |
| `image_topic` | string | `/camera/camera/color/image_raw` | RGB image topic |
| `depth_topic` | string | `/camera/camera/depth/image_rect_raw` | Depth image topic |

### Human Pose Data Format

Published as `sensor_msgs/JointState` with 33 landmarks x 3 coordinates = 99 values:

```
position = [x0, y0, z0, x1, y1, z1, ..., x32, y32, z32]
velocity = [visibility scores]
```

Key landmark indices used by safety monitor:
- 11: left shoulder
- 12: right shoulder
- 15: left wrist
- 16: right wrist

### Launch

```bash
ros2 launch optimo_human_perception human_perception.launch.py
```

Launches both the RealSense camera driver and the perception node.

---

## Safety Monitor Node

**Package:** `optimo_safety_monitor`
**Executable:** `safety_monitor_node`
**Node name:** `safety_monitor` (under `/optimo` namespace)

### ROS Interfaces

| Direction | Topic/Service | Type | Description |
|-----------|--------------|------|-------------|
| **Subscribe** | `/optimo/external_wrench` | `geometry_msgs/WrenchStamped` | End-effector force from controller (1 kHz) |
| **Subscribe** | `/optimo/joint_states` | `sensor_msgs/JointState` | Joint positions for baseline lookup (50 Hz) |
| **Subscribe** | `/optimo/human_pose` | `sensor_msgs/JointState` | Human skeleton from perception node |
| **Publish** | `~/status` | `std_msgs/Bool` | Overall safety: `true` = safe, `false` = danger |
| **Publish** | `~/human_pose_safe` | `std_msgs/Bool` | Pose safety only: `true` = safe, `false` = danger |
| **Service (server)** | `~/record_baseline` | `std_srvs/Trigger` | Start recording baseline (deletes old CSV) |
| **Service (server)** | `~/save_baseline` | `std_srvs/Trigger` | Stop recording, save to CSV |
| **Service (client)** | `/optimo/optimo_effort_controller/stop_cb` | `std_srvs/Trigger` | Calls controller to kill trajectory |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `check_rate_hz` | double | 50.0 | Safety evaluation frequency |
| `wrench_force_threshold` | double | 10.0 | Max delta force (N) before stop |
| `baseline_file` | string | `""` | Path to baseline CSV (empty = no force checking) |

### Safety Check: `is_wrench_safe()`

Compares current external wrench against a **position-dependent baseline**.

**Logic:**
1. If no baseline loaded or no data received yet → return `true` (safe, force checking disabled)
2. Find the baseline sample whose joint positions are nearest (L2 distance) to the current joint state
3. Compute delta: `delta = ||current_wrench - baseline_wrench||` at that configuration
4. If `delta > wrench_force_threshold` → return `false` (unsafe)

**Warning levels:**

| Condition | Log level | Color | Message |
|-----------|-----------|-------|---------|
| delta > 70% of threshold | WARN | Yellow | `WRENCH WARNING: delta=N approaching threshold=T` |
| delta > threshold | ERROR | Red | `WRENCH UNSAFE: delta=N EXCEEDS threshold=T` |

**Why baseline?** The robot's own gravity and dynamics produce non-zero wrench readings that vary with joint configuration. A baseline recorded during normal (uncontacted) trajectory playback captures these expected forces, so only *unexpected* external forces trigger a stop.

### Safety Check: `is_pose_safe()`

Checks if a human is in a dangerous posture relative to the robot.

**Logic:**
1. If no pose data received yet → return `true` (safe)
2. If fewer than 17 landmarks available → return `true` (incomplete skeleton)
3. Extract y-coordinates for left/right shoulders and wrists
4. If **both** wrists are above their respective shoulders → return `false` (unsafe)

**Warning level:**

| Condition | Log level | Color | Message |
|-----------|-----------|-------|---------|
| Both hands raised | WARN | Yellow | `POSE UNSAFE: both hands raised!` |

### Combined Safety Logic

Both checks run every timer tick. The overall safety status is:

```
safe = is_pose_safe() AND is_wrench_safe()
```

Either check can independently trigger an emergency stop. If a data source is not yet available (no baseline loaded, no human detected), that check defaults to `true` so the other check still works independently.

### Stop Behavior

- `stop_cb` is a **full stop** — kills the trajectory and clears the task queue
- **No pause/resume** — operator must manually restart the trajectory
- Stop triggers **once per danger event** (flag prevents repeated calls)
- When readings return to safe, the flag resets for the next event

### Status Topic Summary

The timer callback publishes a periodic log (throttled to every 2s):
```
[timestamp] pose=SAFE wrench=SAFE overall=SAFE delta_force=3.21 N
```

---

## ANTERO Gripper Node

**Package:** `antero_gripper` (planned)
**Node name:** `antero_gripper` (under `/optimo` namespace)

Subscribes to the safety monitor's status topics and controls the ANTERO Gripper via a USB-to-CAN transceiver. Activates the gripper when conditions are safe and deactivates it when an unsafe condition is detected.

### ROS Interfaces

| Direction | Topic/Service | Type | Description |
|-----------|--------------|------|-------------|
| **Subscribe** | `/optimo/safety_monitor/status` | `std_msgs/Bool` | Overall safety status (`false` = unsafe) |
| **Subscribe** | `/optimo/safety_monitor/human_pose_safe` | `std_msgs/Bool` | Pose safety status (`false` = unsafe) |
| **Publish** | `~/grip_status` | `std_msgs/UInt8` | Current gripper status |

### Behavior

- When either subscribed topic publishes `false`, the node sends a deactivate command to the ANTERO Gripper over the USB-to-CAN bus
- When both topics are `true`, the gripper is kept active
- This ensures the gripper releases its grasp during an emergency stop, preventing damage to held objects or the environment
- The node operates independently from the safety monitor's `stop_cb` call to the arm controller — the arm stops and the gripper deactivates in parallel

### Hardware Interface

- **Bus:** USB-to-CAN transceiver (e.g., PCAN-USB, CANable)
- **Protocol:** CAN bus commands to the ANTERO Gripper controller
- **Actions:** Activate (grip) on safe, deactivate (release) on unsafe

---

## Baseline Calibration Workflow

The baseline captures expected wrench readings at each joint configuration during normal trajectory playback (no external contact).

### Step 1: Record Baseline

```bash
# Launch bringup + safety monitor (no baseline_file = force checking disabled)
ros2 launch optimo_bringup optimo.launch.py
ros2 launch optimo_safety_monitor safety_monitor.launch.py

# Start recording (deletes any old baseline CSV)
ros2 service call /optimo/safety_monitor/record_baseline std_srvs/srv/Trigger

# Play the trajectory (no touching the robot!)
ros2 service call /optimo/optimo_effort_controller/play_traj_cb \
  optimo_msgs/srv/PlayTrajCb \
  "{duration: 0, goal: {filepath: '/path/to/traj', param: {imp: {impedance: 5.0, max_force: 5.0, stop_sensitivity: 5.0}, recovery_speed: 5.0, accept_diff_start: true}}}"

# Wait for trajectory to finish, then save
ros2 service call /optimo/safety_monitor/save_baseline std_srvs/srv/Trigger
```

This saves a CSV to `/tmp/optimo_wrench_baseline.csv` (or the path set by `baseline_file`).

**Note:** Calling `record_baseline` deletes the old CSV file first, so a stale baseline is never accidentally used if recording fails.

### Step 2: Run with Baseline

```bash
ros2 launch optimo_safety_monitor safety_monitor.launch.py \
  baseline_file:=/tmp/optimo_wrench_baseline.csv \
  wrench_force_threshold:=20.0
```

Now the monitor will detect external forces during trajectory playback and stop the robot.

### Baseline CSV Format

```
q1,q2,q3,q4,q5,q6,q7,fx,fy,fz
0.193,2.576,0.029,-1.799,-0.087,-0.817,-0.087,1.23,-0.45,8.91
...
```

Each row is one sample: 7 joint angles + 3 force components.

---

## Bringup / OptimoEffortController

The `optimo_bringup` package launches the ros2_control stack with the `OptimoEffortController`.

### Services

| Service | Type | Purpose |
|---------|------|---------|
| `play_traj_cb` | `optimo_msgs/PlayTrajCb` | Start trajectory playback |
| `stop_cb` | `std_srvs/Trigger` | Kill current trajectory + clear queue |
| `free_motion_cb` | `optimo_msgs/GenericCb` | Gravity compensation mode |
| `teach_traj_cb` | `optimo_msgs/StringCb` | Record a new trajectory |
| `move_home_cb` | `optimo_msgs/PosCb` | Move to home configuration |
| `joint_trajectory_cb` | `optimo_msgs/JointTrajectoryCb` | Play joint trajectory from file |
| `moveit_cb` | `optimo_msgs/MoveitCb` | Plan & execute with MoveIt |
| `servo_cb` | `optimo_msgs/ServoCb` | Low-level servo (joint or Cartesian) |
| `calculate_load` | `std_srvs/Trigger` | Enable load calculation |

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/optimo/external_wrench` | `geometry_msgs/WrenchStamped` | 1 kHz | EE force (from `model->get_ee_force()`) |
| `/optimo/joint_states` | `sensor_msgs/JointState` | 50 Hz | Joint positions, velocities, efforts |
| `/optimo/ee_pose_current` | `optimo_msgs/PoseElbow` | — | End-effector pose + elbow angle |
| `/optimo/jacobian` | `std_msgs/Float64MultiArray` | — | 7x6 Jacobian matrix |
| `/optimo/torque_command` | `std_msgs/Float64MultiArray` | — | Commanded joint torques (milli-Nm) |

**Note:** Joint order in `/optimo/joint_states` is not sequential — it arrives as `[joint2, joint5, joint6, joint1, joint4, joint7, joint3]`. All lookups use joint name matching, not index.

---

## File Structure

```
optimo_safety_monitor/
├── CMakeLists.txt
├── package.xml
├── include/optimo_safety_monitor/
│   └── safety_monitor_node.hpp      # Node class + BaselineSample struct
├── src/
│   └── safety_monitor_node.cpp      # All implementation
└── launch/
    └── safety_monitor.launch.py     # Launch with configurable params

optimo_human_perception/
├── package.xml
├── setup.py
├── optimo_human_perception/
│   └── human_perception_node.py     # MediaPipe pose detection
├── launch/
│   └── human_perception.launch.py   # Launches RealSense + perception
└── resource/
    └── pose_landmarker_lite.task    # MediaPipe model file
```

---

## Launch Reference

| Launch file | Package | What it starts |
|-------------|---------|----------------|
| `optimo.launch.py` | `optimo_bringup` | Full robot stack (ros2_control, controllers, state publishers) |
| `safety_monitor.launch.py` | `optimo_safety_monitor` | Safety monitor node |
| `human_perception.launch.py` | `optimo_human_perception` | RealSense camera + pose detection |

### Typical production launch (all safety features):

```bash
# Terminal 1: Robot
ros2 launch optimo_bringup optimo.launch.py

# Terminal 2: Camera + human detection
ros2 launch optimo_human_perception human_perception.launch.py

# Terminal 3: Safety monitor with force baseline
ros2 launch optimo_safety_monitor safety_monitor.launch.py \
  baseline_file:=/tmp/optimo_wrench_baseline.csv \
  wrench_force_threshold:=20.0
```

---

## Future Work

- **Pause/resume:** Currently stop is destructive; could add a traj_player intermediary node for pause/resume capability
- **Multi-joint effort monitoring:** Currently unused, but effort data is available on `/optimo/joint_states`
- **Proximity-based pose safety:** Current pose check uses hand-raise heuristic; could add distance-based human-robot proximity check
