# Launch Files and Configuration

## Launch file inventory

| File | Starts | FastDDS SHM | Use case |
|------|--------|:-----------:|---------|
| `zed_mavros_fc.launch.py` | ZED + MAVROS + bridge | Disabled ✓ | Standard pre-flight on Jetson |
| `mavros_fc.launch.py` | MAVROS + bridge | Disabled ✓ | ZED already running in another terminal |
| `zed.launch.py` | ZED only | — | Camera inspection / recording |
| `zed_mavros_sitl.launch.py` | ZED + MAVROS + bridge | **Not set** | SITL (requires manual workaround below) |

## Launch arguments

`zed_mavros_fc.launch.py` and `mavros_fc.launch.py` accept:

| Argument | Default | Notes |
|----------|---------|-------|
| `fcu_url` | `/dev/ttyTHS1:921600` | Jetson Telem2 UART; use `tcp://127.0.0.1:5760` for SITL |
| `camera_model` | `zed2i` | ZED model string for `zed_wrapper` |
| `zed_odom_topic` | `/zed/zed_node/odom` | Forwarded to bridge node parameter |

## FastDDS shared-memory issue

After MAVROS crashes or restarts, stale type-signature entries in `/dev/shm/fastrtps_*` cause topics to appear but carry no data. The production launch files set `FASTRTPS_DEFAULT_PROFILES_FILE` to `config/fastdds_no_shm.xml` which disables SHM transport.

`zed_mavros_sitl.launch.py` does **not** set this. Workaround:

```bash
# Option A — manual export before launch
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix sky_vision2)/share/sky_vision2/config/fastdds_no_shm.xml
ros2 launch sky_vision2 zed_mavros_sitl.launch.py fcu_url:=tcp://127.0.0.1:5760

# Option B — clear stale SHM entries
rm -f /dev/shm/fastrtps_*
ros2 launch sky_vision2 zed_mavros_sitl.launch.py fcu_url:=tcp://127.0.0.1:5760
```

## MAVROS plugin allowlist (`config/apm_pluginlists_vision.yaml`)

Only essential plugins are loaded to reduce overhead:

```
sys_status, sys_time, command, local_position, global_position,
home_position, imu, mocap_pose_estimate, vision_speed
```

`mocap_pose_estimate` consumes `/mavros/mocap/pose` → `ATT_POS_MOCAP` MAVLink message (full quaternion, avoids Eigen yaw-folding bug in `vision_pose`).
`vision_speed` consumes `/mavros/vision_speed/speed_twist` → `VISION_SPEED_ESTIMATE` MAVLink message.

## Required ArduPilot FCU parameters

Set these on the Pixhawk before any flight that uses visual odometry:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `EK3_SRC1_POSXY` | `6` | ExternalNav horizontal position |
| `EK3_SRC1_VELXY` | `6` | ExternalNav horizontal velocity |
| `EK3_SRC1_POSZ` | `1` | Barometer vertical position |
| `EK3_SRC1_VELZ` | `0` | No vertical velocity source |
| `EK3_SRC1_YAW` | `6` | ExternalNav yaw |
| `VISO_TYPE` | `1` | Enable visual odometry |

Without these, MAVROS publishes vision messages but ArduPilot silently ignores them.

## Startup verification

After launching, in a new terminal with `export ROS_DOMAIN_ID=42`:

```bash
ros2 topic echo /mavros/state --once          # connected: True
ros2 topic hz /zed/zed_node/odom             # ~30 Hz (after ~15 s warm-up)
ros2 topic hz /mavros/vision_pose/pose       # ~30 Hz
ros2 topic hz /mavros/vision_speed/speed_twist  # ~30 Hz
# Bridge log must show: "HOME SET from vision EKF — ready to arm"
```

## Workspace

This package lives in `~/sky_ws2/src/sky_vision2/` as a git submodule tracking the `imav_2026` branch on `github.com/SkyRats/sky_vision2`. Changes must be committed and pushed from the submodule directory, then the workspace's submodule pointer updated.
