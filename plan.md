# Plan: Safety Monitor Node for Trajectory Playback

## Summary

Create `optimo_safety_monitor` — a single ROS 2 node that monitors safety conditions during trajectory playback and stops the robot when danger is detected.

```
                    ┌──────────────────────────┐
  Inputs            │   SafetyMonitorNode      │         Outputs
                    │                          │
  /external_wrench ─┤►  is_force_safe()        │
  (WrenchStamped)   │         ↓                ├──► ~/status (Bool)
                    │   evaluate_safety()       │    [safe=true / danger=false]
  /human_pose ──────┤►  is_pose_safe()         │
  (JointState)      │         ↓                ├──► stop_cb service call
                    │   if unsafe → STOP       │    [kills trajectory on bringup]
                    └──────────────────────────┘
```

**Current version:** Both safety checks are stubs returning `true`. User replaces with real logic later.

## Files to Create

```
optimo_safety_monitor/
├── CMakeLists.txt
├── package.xml
├── include/optimo_safety_monitor/safety_monitor_node.hpp
├── src/safety_monitor_node.cpp
└── launch/safety_monitor.launch.py
```

## Node Interface

| Direction | Name | Type | Purpose |
|-----------|------|------|---------|
| Subscribe | `/optimo/external_wrench` | `WrenchStamped` | EE force data (from bringup) |
| Subscribe | `/optimo/human_pose` | `JointState` | Human skeleton joints (from pose estimator) |
| Publish | `~/status` | `Bool` | `true`=safe, `false`=danger |
| Service client | `/optimo/optimo_effort_controller/stop_cb` | `Trigger` | Stops trajectory playback |

## Parameters

| Parameter | Type | Default | Purpose |
|-----------|------|---------|---------|
| `check_rate_hz` | double | 50.0 | Safety evaluation frequency |
| `force_threshold` | double | 30.0 | Max EE force (N) before stop |

## Implementation

- Timer callback at `check_rate_hz` calls `is_force_safe()` and `is_pose_safe()`
- Both stubs return `true` with TODO comments
- If either returns `false`: async call `stop_cb`, publish `status=false`, log warning
- Otherwise: publish `status=true`
- After triggering stop once, don't spam — set a flag to avoid repeated calls

## Usage

```bash
# Terminal 1: Bringup
ros2 launch optimo_bringup optimo.launch.py

# Terminal 2: Safety monitor
ros2 launch optimo_safety_monitor safety_monitor.launch.py

# Terminal 3: Play trajectory
ros2 service call /optimo/optimo_effort_controller/play_traj_cb \
  optimo_msgs/srv/PlayTrajCb \
  "{duration: 0, goal: {filepath: '/path/to/traj', param: {imp: {impedance: 5.0, max_force: 5.0, stop_sensitivity: 5.0}, recovery_speed: 5.0, accept_diff_start: true}}}"
```

## Verification

1. `colcon build --packages-select optimo_safety_monitor`
2. Launch bringup + safety monitor
3. Confirm `~/status` publishes `true`
4. Flip a stub to return `false` → confirm `stop_cb` is called and logged
5. Play a real trajectory with stubs returning `true` → confirm uninterrupted playback
