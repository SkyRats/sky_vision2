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

# Full hardware stack — with mavp2p (recommended)
ros2 launch sky_vision2 mavros_mavp2p_fc.launch.py

# Full hardware stack — ZED + MAVROS + bridge (direct serial, no mavp2p)
ros2 launch sky_vision2 zed_mavros_fc.launch.py

# MAVROS + bridge only (ZED already running, no mavp2p)
ros2 launch sky_vision2 mavros_fc.launch.py

# Offline test — no hardware needed
ros2 run sky_vision2 zed_mavros_bridge   # terminal 1
ros2 run sky_vision2 test_zed_odom       # terminal 2
```

Wait for: `Vision data flowing — ready to arm once EKF converges` (bridge log), then confirm `FCU: EKF3 IMU0 is using external nav data` in the MAVROS log before arming. Home is set on the FC by `ekf_set_home.lua` (see below) — wait for its `ekf_home: HOME SET from vision EKF — ready to arm` line on the GCS before arming too.

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

Confirmed correct in a live hardware run (EKF3 yaw-aligned, using external nav data). The module docstring/startup log previously described a different transform ("negate Y; flip qy,qz") than `_odom_cb` actually executed — fixed 2026-07-01, so they now agree.

**No vision_speed:** the ZED wrapper's `publishOdom()` never fills the Odometry message's `twist` field, so velocity is always exactly zero regardless of real motion. The bridge does not publish vision_speed at all (removed 2026-07-01) rather than forward a false zero-velocity measurement — `EK3_SRC1_VELXY` must be `0` (None), not `6`, on the FC. See `docs/zed_mavros_bridge.md` at the workspace root for the full writeup.

`.claude/rules/yaw_frame_research.md` describes an **older/superseded architecture** (MAVROS auto ENU→NED via `vision_pose_estimate`, +π/2 quaternion offset) that no longer matches this file — kept for historical background only.

### Critical: QoS

ZED publishes odom with **BEST_EFFORT** reliability. The bridge subscription must match — ROS2 silently drops mismatched QoS connections.

### EKF home-setting: Lua on the FC

Home is set by `indoor_2026/fc_scripts/ekf_set_home.lua`, run onboard the FC. It monitors EKF3 health and calls `vehicle:set_home_to_current_location()` once vision has been stable ~5 s, printing `ekf_home: HOME SET from vision EKF — ready to arm` to the GCS Messages tab. **Do not arm before that line.**

**Requires an SD card in the Pixhawk 6C** — ArduPilot loads Lua from `APM/scripts/` on the card, and needs `SCR_ENABLE=1`. No card → no scripting → no home-set → the drone will not arm. Keep the drone **stationary for ~20 s** after launch so the EKF converges before the script's stability window.

> **Removed 2026-07-08:** a ROS-side fallback node, `ekf_home_watchdog`, previously reproduced this gating and called `/mavros/mavros/set_home` when no SD card was available. It was deleted in favour of doing home-setting on the FC via Lua. It never verified end-to-end anyway — a live test got `MAV_RESULT_FAILED`, likely because the EKF *origin* (distinct from home) wasn't established in that session. If you need home set without an SD card, do it manually from a GCS.

### FastDDS shared-memory

After MAVROS crashes or restarts, stale `/dev/shm/fastrtps_*` entries cause topics to appear active but carry no data. `config/fastdds_no_shm.xml` disables SHM transport — `zed_mavros_fc.launch.py` and `mavros_fc.launch.py` set this automatically. `zed_mavros_sitl.launch.py` does **not** — clear manually with `rm -f /dev/shm/fastrtps_*` or use `mavros_fc.launch.py fcu_url:=tcp://127.0.0.1:5760` for SITL.

### Bridge exclusivity

`sky_vision2` is the only package that should run `zed_mavros_bridge`. **Never run two instances simultaneously** — duplicate messages on `/mavros/vision_pose/pose` corrupt the EKF. Verify: `ros2 node list | grep zed_mavros_bridge` must show exactly one.

## Launch files

| File | Starts | Notes |
|------|--------|-------|
| `mavros_mavp2p_fc.launch.py` | mavp2p + MAVROS + bridge | **Recommended** — fans FC link to 14551/14552 |
| `zed_mavros_fc.launch.py` | ZED + MAVROS + bridge | Direct serial, no mavp2p |
| `mavros_fc.launch.py` | MAVROS + bridge | Direct serial, no ZED |
| `zed.launch.py` | ZED only | Camera test |

### mavros_mavp2p_fc arguments

| Argument | Default | Notes |
|----------|---------|-------|
| `mavp2p_source` | `serial:/dev/ttyACM0:921600` | USB ACM; use `serial:/dev/ttyTHS1:921600` for UART or `tcpc://127.0.0.1:5760` for SITL |
| `zed_odom_topic` | `/zed/zed_node/odom` | ZED odom topic |

MAVROS connects to mavp2p at `udp://127.0.0.1:0@127.0.0.1:14551`. The `sky_navigation`
`Drone` class connects to `udpout:127.0.0.1:14552`. Both endpoints are UDP servers
(`udps`) so each client heartbeats first and mavp2p learns the return address.

### zed_mavros_fc / mavros_fc arguments

| Argument | Default | Notes |
|----------|---------|-------|
| `fcu_url` | `/dev/ttyTHS1:921600` | Jetson Telem2 UART; use `tcp://127.0.0.1:5760` for SITL |
| `camera_model` | `zed2i` | ZED model string |
| `zed_odom_topic` | `/zed/zed_node/odom` | ZED odom topic |

### MAVROS plugin allowlist (`config/apm_pluginlists_vision.yaml`)

Active plugins: `sys_status`, `sys_time`, `command`, `local_position`, `global_position`,
`home_position`, `imu`, `vision_pose`

`setpoint_position` and `setpoint_velocity` are intentionally **absent** — all movement
commands go via pymavlink through mavp2p:14552, not via MAVROS topics. Adding them back
creates a dual-commander risk.

## Required ArduPilot FCU parameters

| Parameter | Value |
|-----------|-------|
| `EK3_SRC1_POSXY` | `6` (ExternalNav) |
| `EK3_SRC1_VELXY` | `0` (None — no vision_speed is published) |
| `EK3_SRC1_POSZ` | `1` (Baro) |
| `EK3_SRC1_VELZ` | `0` (None) |
| `EK3_SRC1_YAW` | `6` (ExternalNav) |
| `VISO_TYPE` | `1` |

## See also

- `.claude/rules/bridge_node.md` — full topic/parameter reference, NED alignment offset
- `indoor_2026/fc_scripts/ekf_set_home.lua` — FC-side Lua that sets home from the vision EKF (requires SD card)
- `.claude/rules/yaw_frame_research.md` — full ZED→MAVROS→ArduPilot frame chain, ±π safety, EKF3 fusion
- `.claude/rules/launch_and_config.md` — launch variants, FastDDS, config files
- `~/sky_ws2/CLAUDE.md` — workspace context (build, SITL workflow, submodule management)
