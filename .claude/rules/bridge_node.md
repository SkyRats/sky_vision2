# ZedMavrosBridge — Node Reference

## Node identity

- **Package:** `sky_vision2`
- **Executable:** `zed_mavros_bridge` (entry point: `sky_vision2.zed_mavros_bridge:main`)
- **Source:** `sky_vision2/zed_mavros_bridge.py`

## Topics

| Direction | Topic | Type | QoS |
|-----------|-------|------|-----|
| Subscribes | `zed_odom_topic` param (default `/zed/zed_node/odom`) | `nav_msgs/Odometry` | **BEST_EFFORT**, VOLATILE, depth=10 |
| Publishes | `mavros_vision_pose_topic` param (default `/mavros/mavros/pose`) | `geometry_msgs/PoseStamped` | default (RELIABLE) |

**Critical:** ZED driver publishes odom with BEST_EFFORT. The bridge subscription must use BEST_EFFORT — using RELIABLE silently receives nothing.

**No vision_speed publisher (removed 2026-07-01).** The ZED wrapper's `publishOdom()` never fills `twist`, so `msg.twist.twist.linear` is always exactly zero regardless of real motion. Forwarding that as `VISION_SPEED_ESTIMATE` would tell the EKF "velocity = 0" confidently even during real motion — worse than omitting it. `vision_speed` was also removed from `config/apm_pluginlists_vision.yaml`'s plugin allowlist. `EK3_SRC1_VELXY` must be `0` (None) on the FC, not `6`.

**No `/mavros/estimator_status` subscription and no `set_home` service client exist in this file** (verified against `sky_vision2/zed_mavros_bridge.py` 2026-07-01) — that logic was removed from the bridge. Home-setting is now done on the FC by `indoor_2026/fc_scripts/ekf_set_home.lua` (Lua). (A ROS-side `ekf_home_watchdog` node briefly held this logic; it was removed 2026-07-08 in favour of the Lua script.)

### Why the default topic is `/mavros/mavros/pose`, not `/mavros/vision_pose/pose`

The MAVROS `vision_pose` plugin subscribes to a **relative** topic (`~pose`). Since the `mavros_node` in this launch file runs with `name='mavros'` (no explicit `namespace:=` override), its effective node namespace is `/mavros/mavros`, so the plugin's relative subscription resolves to `/mavros/mavros/pose` — not the `/mavros/vision_pose/pose` shown in generic MAVROS documentation/examples.

Verify with `ros2 node info /mavros/mavros` if this ever needs re-checking (e.g. after a MAVROS/launch-file version bump).

## Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `zed_odom_topic` | `/zed/zed_node/odom` | Change if using a non-standard ZED namespace |
| `mavros_vision_pose_topic` | `/mavros/mavros/pose` | Feeds EKF3 position |
| `yaw_offset_rad` | `0.0` (source default) — launch file sets `-1.5708` | Zeroes ZED's initial heading; applied as a pure-Z quaternion left-multiply after the axis remap |

## Frame convention (verified against running code, 2026-07-01)

`_odom_cb` performs its own axis remap — it does **not** rely on MAVROS auto-converting ENU→NED:

```python
# position: x_ned = -y_zed, y_ned = x_zed, z_ned = z_zed
# orientation: qx,qy,qz,qw passed through unchanged, then rotated by yaw_offset_rad
#              (q_out = q_corr * q_in, q_corr = pure Z rotation of yaw_offset_rad)
```

Confirmed operationally correct in a live hardware run (EKF3 aligned yaw, used external nav data, position held near zero while stationary). The module docstring/startup log previously described a different transform ("negate Y, flip qy/qz") than `_odom_cb` actually executed — fixed 2026-07-01 alongside the vision_speed removal, so they now agree.

`.claude/rules/yaw_frame_research.md` describes an **older/superseded architecture** (MAVROS auto ENU→NED via `vision_pose_estimate` plugin's built-in conversion, +π/2 quaternion offset applied by the bridge). That no longer matches the current `_odom_cb` implementation — kept for historical background only, not as a behavior reference.

## EKF home-setting is done on the FC (Lua), not in this bridge

Earlier versions of this bridge subscribed to `/mavros/estimator_status` and called `/mavros/mavros/set_home` once EKF health looked good. Both were removed from `zed_mavros_bridge.py` because `/mavros/estimator_status` is fed by ArduPilot's `EKF_STATUS_REPORT` MAVLink message at whatever rate `SR2_EXTRA3` specifies — defaults to `0` on Telem2, so the topic never actually publishes new data.

Home-setting is now done on the flight controller by `indoor_2026/fc_scripts/ekf_set_home.lua` (requires an SD card + `SCR_ENABLE=1`). A short-lived ROS-side `ekf_home_watchdog` node held this logic between 2026-07-01 and 2026-07-08, then was removed in favour of the Lua script.

**Keep the drone stationary for the first ~20 s after launch** so the EKF converges cleanly before the Lua script's stability window starts counting.

## Bridge exclusivity

`sky_vision2` is the only package that should run `zed_mavros_bridge`. Never run two instances simultaneously — both would publish to `/mavros/vision_pose/pose` and corrupt the EKF.

```bash
ros2 node list | grep zed_mavros_bridge   # must show exactly one
```

## test_zed_odom utility

`sky_vision2/test_zed_odom.py` runs two nodes in one process:
- **`ZedOdomPublisher`** — synthetic circular odom at 30 Hz on `/zed/zed_node/odom` (BEST_EFFORT)
- **`BridgeVerifier`** — subscribes to MAVROS output topics, logs counts every 5 s
