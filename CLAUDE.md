# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this package does

`sky_vision2` is the **production vision stack** for the SkyRats IMAV 2026 drone. It bridges ZED2i camera odometry into ArduPilot's EKF3 via MAVROS, providing visual odometry-based position and velocity estimates for indoor GPS-denied flight.

Lives in `~/sky_ws2/src/sky_vision2/` — the only workspace.

## Executables

| Executable | Source | Purpose |
|-----------|--------|---------|
| `zed_mavros_bridge` | `sky_vision2/zed_mavros_bridge.py` | Production bridge: ZED odom → MAVROS pose + velocity, EKF watchdog, auto set_home |
| `test_zed_odom` | `sky_vision2/test_zed_odom.py` | Synthetic circular-trajectory odom publisher for offline testing |

## Build and run

```bash
cd ~/sky_ws2
colcon build --packages-select sky_vision2
source install/setup.bash

# Full hardware stack
ros2 launch sky_vision2 zed_mavros_fc.launch.py

# Bridge + MAVROS only (ZED running separately)
ros2 launch sky_vision2 mavros_fc.launch.py

# ZED only
ros2 launch sky_vision2 zed.launch.py

# Offline test (no hardware)
ros2 run sky_vision2 zed_mavros_bridge   # terminal 1
ros2 run sky_vision2 test_zed_odom       # terminal 2
```

All terminals need `export ROS_DOMAIN_ID=42`.

## Run tests

```bash
colcon test --packages-select sky_vision2
colcon test-result --verbose
```

## Architecture: ZedMavrosBridge

### Topics

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/zed/zed_node/odom` | `nav_msgs/Odometry` | BEST_EFFORT QoS — must match ZED driver |
| Sub | `/mavros/estimator_status` | `mavros_msgs/EstimatorStatus` | EKF health, for auto-home |
| Pub | `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | → `VISION_POSITION_ESTIMATE` → EKF3 |
| Pub | `/mavros/vision_speed/speed_twist` | `geometry_msgs/TwistStamped` | → `VISION_SPEED_ESTIMATE` → EKF3 |
| Service | `/mavros/cmd/set_home` | `mavros_msgs/CommandHome` | Called once after 5s stable EKF |

### QoS — critical detail

The ZED wrapper publishes odom with **BEST_EFFORT** reliability. The bridge subscription must use the same profile or it will receive no messages (ROS2 silently drops incompatible QoS matches):

```python
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10,
)
```

### Frame correction

**MAVROS with ArduPilot (`apm.launch`) does NOT convert ENU→NED.** The `vision_pose` plugin passes `PoseStamped` data directly as `VISION_POSITION_ESTIMATE`. ArduPilot EKF3 expects NED (X=North, Y=East, Z=Down). The bridge must publish NED.

ZED odom frame (hardware-observed): X=North, Y=West, Z=Down. Only Y needs negation:

```python
position.x =  zed.position.x   # North, unchanged
position.y = -zed.position.y   # West → East
position.z =  zed.position.z   # Down, unchanged

# Quaternion: flipping Y reverses rotations around Y (pitch) and Z (yaw)
q.x =  qx   # unchanged
q.y = -qy   # pitch sign flips
q.z = -qz   # yaw sign flips
q.w =  qw
```

See `.claude/rules/bridge_node.md` for full details.

### EKF home watchdog

The bridge monitors `/mavros/estimator_status.pos_horiz_rel`. Once it stays `True` for 5 continuous seconds, it calls `set_home`. If the flag drops the countdown resets. The log line `"HOME SET from vision EKF — ready to arm"` confirms success. Keep the drone **stationary for the first ~20 s** after launch.

## Launch file matrix

| File | ROS_DOMAIN_ID | FastDDS no-SHM | ZED | MAVROS | Bridge |
|------|:---:|:---:|:---:|:---:|:---:|
| `zed_mavros_fc.launch.py` | 42 | YES | yes | yes | yes |
| `mavros_fc.launch.py` | 42 | YES | — | yes | yes |
| `zed.launch.py` | 42 | — | yes | — | — |
| `zed_mavros_sitl.launch.py` | 42 | NO | yes | yes | yes |

`zed_mavros_sitl.launch.py` does **not** set `FASTRTPS_DEFAULT_PROFILES_FILE`. Before restarting MAVROS under this launch file, clear stale SHM entries:

```bash
rm -f /dev/shm/fastrtps_*
```

Or set it in the terminal before launch:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix sky_vision2)/share/sky_vision2/config/fastdds_no_shm.xml
```

## Config files

### `config/apm_pluginlists_vision.yaml`
MAVROS plugin allowlist — loads only the plugins needed for visual odometry.

### `config/fastdds_no_shm.xml`
Disables DDS shared-memory transport. Prevents stale type-signature entries in `/dev/shm` from causing topics to silently carry no data after MAVROS restarts.

## Startup verification

```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /mavros/state --once          # connected: True
ros2 topic hz /zed/zed_node/odom             # ~30 Hz (appears after ~15 s)
ros2 topic hz /mavros/vision_pose/pose       # ~30 Hz
ros2 topic hz /mavros/vision_speed/speed_twist  # ~30 Hz
# Bridge log: "HOME SET from vision EKF — ready to arm"
```

## Never run sky_vision2 and indoor_2026 bridges simultaneously

Both `ZedMavrosBridge` (this package) and `pose_relay` (`indoor_2026`) publish on `/mavros/vision_pose/pose`. Running both produces duplicate messages that confuse the EKF. Only one bridge should be active at any time.

## See also

- `.claude/rules/bridge_node.md` — full node API, frame math, EKF watchdog
- `.claude/rules/launch_and_config.md` — launch variants, FastDDS, FCU parameters
- `~/sky_ws2/CLAUDE.md` — workspace context
