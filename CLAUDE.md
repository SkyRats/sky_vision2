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

Wait for: `Vision data flowing — ready to arm once EKF converges` (bridge log), then confirm `FCU: EKF3 IMU0 is using external nav data` in the MAVROS log before arming. `ekf_home_watchdog` calls `/mavros/mavros/set_home` automatically once vision has been flowing stably for a few seconds — wait for its `HOME SET from vision EKF — ready to arm` log line too.

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
| `ekf_home_watchdog` | `sky_vision2/ekf_home_watchdog.py` | Sets FC home via `/mavros/mavros/set_home` once vision is stable — runs during every flight |
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

### EKF home-setting: Lua (primary) vs ekf_home_watchdog (fallback)

**Primary/trusted mechanism:** `indoor_2026/fc_scripts/ekf_set_home.lua`, run onboard the FC. This is what the team actually relies on — the drone flies with it. **Currently not deployed on this specific Pixhawk 6C: it has no SD card, and ArduPilot loads Lua scripts from `APM/scripts/` on the SD card.** Deploy it once an SD card is installed.

**Fallback:** a ROS2 node, `ekf_home_watchdog`, added 2026-07-01 as a stand-in for when no SD card is available. It watches `/mavros/mavros/pose` for vision to start moving and stay stable, then calls `/mavros/mavros/set_home` (`current_gps=True`) once. Verified 2026-07-01: both this service call and the Lua script's `vehicle:set_home_to_current_location()` resolve to the exact same underlying ArduPilot function (`Copter::set_home_to_current_location`, `ArduCopter/commands.cpp`) — so this node is not a weaker substitute for the *set_home call itself*, only for the *gating logic* around it (see next paragraph). See `.claude/rules/ekf_home_watchdog.md`.

`ekf_home_watchdog` can't use `/mavros/estimator_status` (dead — `SR2_EXTRA3=0` on Telem2) or true EKF variance (Lua reads `ahrs:get_variances()` directly in C++; nothing exposes that over MAVLink here), so it substitutes a position-jump check on `/mavros/mavros/pose` — in practice this ends up watching the bridge's own outgoing vision pose rather than an independent FC-side echo, since `local_position` plugin's own outputs aren't streaming on this hardware (`SR2_POSITION` likely `0` on Telem2, same issue class as `SR2_EXTRA3`). This is weaker than Lua's check — it can't detect slow EKF drift, only sudden discontinuities. Keep the drone **stationary for ~20 s** after launch regardless.

**Not independently verified end-to-end on this hardware/session:** a live test 2026-07-01 got `MAV_RESULT_FAILED` calling `set_home` (likely because the EKF origin — a separate concept from home, see `ArduCopter/commands.cpp`'s `set_home()` — was never established in that particular session: no GPS fix, no beacon, and ExternalNav fusion doesn't set origin on its own). The team's real flights work, so origin evidently does get established somehow in practice (GPS fix, a manual GCS "set origin" step, or something not yet identified) — this just wasn't reproduced in that test session. Treat `ekf_home_watchdog` as unverified until confirmed on a real flight.

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
| `EK3_SRC1_VELXY` | `0` (None — no vision_speed is published) |
| `EK3_SRC1_POSZ` | `1` (Baro) |
| `EK3_SRC1_VELZ` | `0` (None) |
| `EK3_SRC1_YAW` | `6` (ExternalNav) |
| `VISO_TYPE` | `1` |

## See also

- `.claude/rules/bridge_node.md` — full topic/parameter reference, NED alignment offset
- `.claude/rules/ekf_home_watchdog.md` — home-setting node: topics, gating logic, parameters
- `.claude/rules/yaw_frame_research.md` — full ZED→MAVROS→ArduPilot frame chain, ±π safety, EKF3 fusion
- `.claude/rules/launch_and_config.md` — launch variants, FastDDS, config files
- `~/sky_ws2/CLAUDE.md` — workspace context (build, SITL workflow, submodule management)
