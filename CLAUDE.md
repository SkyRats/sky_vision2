# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this package does

`sky_vision2` bridges ZED2i camera odometry into ArduPilot's EKF3 via MAVROS for GPS-denied indoor flight. It lives in `~/sky_ws2/src/sky_vision2/` as a git submodule (`imav_2026` branch).

```
ZED2i  →  /zed/zed_node/odom  →  ZedMavrosBridge  →  MAVROS  →  ArduPilot EKF3
                                    (NED-align offset)   ENU→NED    (ExternalNav)
```

## Build and run

```bash
cd ~/sky_ws2
colcon build --packages-select sky_vision2
source install/setup.bash
export ROS_DOMAIN_ID=42

# Full hardware stack
ros2 launch sky_vision2 zed_mavros_fc.launch.py

# MAVROS + bridge only (ZED already running)
ros2 launch sky_vision2 mavros_fc.launch.py

# Offline test — no hardware needed
ros2 run sky_vision2 zed_mavros_bridge   # terminal 1
ros2 run sky_vision2 test_zed_odom       # terminal 2
```

Wait for: `Vision data flowing — ready to arm once EKF converges` (bridge log), then confirm `FCU: EKF3 IMU0 is using external nav data` in the MAVROS log before arming. There is no explicit `set_home` call — ArduPilot sets the EKF origin automatically once vision data arrives.

## Tests

Only linting tests exist (no logic unit tests):

```bash
colcon test --packages-select sky_vision2
colcon test-result --verbose
```

## Architecture

### Executables

| Executable | Source | Role |
|-----------|--------|------|
| `zed_mavros_bridge` | `sky_vision2/zed_mavros_bridge.py` | Production bridge — runs during every flight |
| `test_zed_odom` | `sky_vision2/test_zed_odom.py` | Synthetic circular odom at 30 Hz for offline testing |

### Frame convention (verified against running code, 2026-07-01)

The bridge does its own axis remap in `_odom_cb` — it does **not** rely on MAVROS's ENU→NED auto-conversion:

```python
# position: x_ned = -y_zed, y_ned = x_zed, z_ned = z_zed
# orientation: passed through unchanged, then rotated by yaw_offset_rad (pure Z quaternion, left-multiply)
```

`yaw_offset_rad` is a launch parameter (default `-1.5708`, i.e. −90°, set in `zed_mavros_fc.launch.py`) used to zero out the ZED's initial heading at boot.

**Known inconsistency (unresolved, not yet fixed in code):** the module docstring and the node's own startup log message in `zed_mavros_bridge.py` describe a *different* transform ("negate Y; flip qy,qz") than what `_odom_cb` actually executes (swap+negate X/Y, quaternion passthrough). Don't trust the in-file docstring/log text over the actual `_odom_cb` body — verify against source if behavior seems off.

`.claude/rules/yaw_frame_research.md` describes an **older/superseded architecture** (MAVROS auto ENU→NED via `vision_pose_estimate`, +π/2 quaternion offset) that no longer matches this file — kept for historical background only.

### Critical: QoS

ZED publishes odom with **BEST_EFFORT** reliability. The bridge subscription must match — ROS2 silently drops mismatched QoS connections.

### EKF home watchdog

The bridge monitors `/mavros/estimator_status.pos_horiz_rel`. Once `True` for 5 continuous seconds, calls `set_home`. Keep the drone **stationary for ~20 s** after launch.

### FastDDS shared-memory

After MAVROS crashes or restarts, stale `/dev/shm/fastrtps_*` entries cause topics to appear active but carry no data. `config/fastdds_no_shm.xml` disables SHM transport — `zed_mavros_fc.launch.py` and `mavros_fc.launch.py` set this automatically. `zed_mavros_sitl.launch.py` does **not** — clear manually with `rm -f /dev/shm/fastrtps_*` or use `mavros_fc.launch.py fcu_url:=tcp://127.0.0.1:5760` for SITL.

### Bridge exclusivity

`sky_vision2` is the only package that should run `zed_mavros_bridge`. **Never run two instances simultaneously** — duplicate messages on `/mavros/vision_pose/pose` corrupt the EKF. Verify: `ros2 node list | grep zed_mavros_bridge` must show exactly one.

## Launch arguments

| Argument | Default | Notes |
|----------|---------|-------|
| `fcu_url` | `/dev/ttyTHS1:921600` | Jetson Telem2 UART; use `tcp://127.0.0.1:5760` for SITL |
| `camera_model` | `zed2i` | ZED model string |
| `zed_odom_topic` | `/zed/zed_node/odom` | ZED odom topic |
| `yaw_offset_rad` | `-1.5708` (−90°) | Zeroes ZED's initial heading — see Frame convention below |

## Required ArduPilot FCU parameters

| Parameter | Value |
|-----------|-------|
| `EK3_SRC1_POSXY` | `6` (ExternalNav) |
| `EK3_SRC1_VELXY` | `6` (ExternalNav) |
| `EK3_SRC1_POSZ` | `1` (Baro) |
| `EK3_SRC1_VELZ` | `0` (None) |
| `EK3_SRC1_YAW` | `6` (ExternalNav) |
| `VISO_TYPE` | `1` |

## See also

- `.claude/rules/bridge_node.md` — full topic/parameter reference, NED alignment offset, EKF watchdog
- `.claude/rules/yaw_frame_research.md` — full ZED→MAVROS→ArduPilot frame chain, ±π safety, EKF3 fusion
- `.claude/rules/launch_and_config.md` — launch variants, FastDDS, config files
- `~/sky_ws2/CLAUDE.md` — workspace context (build, SITL workflow, submodule management)
