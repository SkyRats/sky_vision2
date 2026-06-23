# ZedMavrosBridge — Node Reference

## Node identity

- **Package:** `sky_vision2`
- **Executable:** `zed_mavros_bridge` (entry point: `sky_vision2.zed_mavros_bridge:main`)
- **Source:** `sky_vision2/zed_mavros_bridge.py`

## Topics

| Direction | Topic | Type | QoS |
|-----------|-------|------|-----|
| Subscribes | `zed_odom_topic` param (default `/zed/zed_node/odom`) | `nav_msgs/Odometry` | BEST_EFFORT, VOLATILE, depth=10 |
| Subscribes | `/mavros/estimator_status` | `mavros_msgs/EstimatorStatus` | RELIABLE (default) |
| Publishes | `mavros_vision_pose_topic` param (default `/mavros/mocap/pose`) | `geometry_msgs/PoseStamped` | RELIABLE (default) |
| Publishes | `mavros_vision_speed_topic` param (default `/mavros/vision_speed/speed_twist`) | `geometry_msgs/TwistStamped` | RELIABLE (default) |
| Service client | `/mavros/cmd/set_home` | `mavros_msgs/srv/CommandHome` | async |

**Critical:** ZED driver publishes odom with BEST_EFFORT. The bridge subscription must use BEST_EFFORT — using RELIABLE silently receives nothing.

## Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `zed_odom_topic` | `/zed/zed_node/odom` | Change if using a non-standard ZED namespace |
| `mavros_vision_pose_topic` | `/mavros/mocap/pose` | Feeds EKF3 position via `mocap_pose_estimate` plugin |
| `mavros_vision_speed_topic` | `/mavros/vision_speed/speed_twist` | Feeds EKF3 velocity |

## CRITICAL: MAVROS does NOT convert ENU→NED for ArduPilot

When using MAVROS with ArduPilot (APM mode), the `mocap_pose_estimate` plugin passes `PoseStamped` data **directly** as `ATT_POS_MOCAP` without any ENU→NED conversion. ArduPilot EKF3 expects NED (X=North, Y=East, Z=Down).

**The bridge must publish NED, not ENU.** Do not assume MAVROS will handle the conversion.

## Why `mocap_pose_estimate` instead of `vision_pose`

The `vision_pose` plugin extracts yaw from the incoming quaternion using Eigen's `eulerAngles(2,1,0)`, which returns yaw only in [0, π]. Any heading past 180° (West side of compass) folds back toward 0° instead of wrapping to negative — making the EKF think the drone faces East when it faces West.

`mocap_pose_estimate` sends `ATT_POS_MOCAP` with a full quaternion (`q[4]`), bypassing the Euler extraction entirely. ArduPilot's `AP_ExternalNav_MAV` backend handles both messages identically — same EKF3 source parameters apply.

## Frame transformation

ZED is mounted **inverted** on the drone. Observed hardware axes:

| Axis | ZED direction | NED target | Correction |
|------|--------------|------------|-----------|
| X | North | North | none |
| Y | West | East | negate Y |
| Z | Up | Down | negate Z |

Flipping Y and Z together is a 180° rotation around X (proper rotation, det = +1).
Quaternion: `q' = (qx, -qy, -qz, qw)` — negate qy and qz, leave qx and qw unchanged.

### Yaw

Yaw is sent as a full quaternion via `ATT_POS_MOCAP`. The range is [-π, π] (full compass coverage). Position and velocity are verified correct on hardware; quaternion yaw fix awaits hardware verification.

## EKF watchdog and auto-home

The node monitors `/mavros/estimator_status`. Once `pos_horiz_rel=True` holds for `STABLE_SECS=5.0` seconds, it calls `set_home` asynchronously. Log output:

```
EKF healthy — starting home countdown
HOME SET from vision EKF — ready to arm
```

Keep the drone **stationary** for the first ~20 s after launch so the EKF converges and the home point is set from the correct position.

## Bridge exclusivity

Both `sky_vision2` and `indoor_2026` expose a `zed_mavros_bridge` executable. Never run both simultaneously — they publish to the same MAVROS pose topic and will corrupt the EKF. Check before launching:

```bash
ros2 node list | grep zed_mavros_bridge
# must show exactly one
```

## test_zed_odom utility

`sky_vision2/test_zed_odom.py` contains two nodes run in one process:
- **`ZedOdomPublisher`** — publishes synthetic circular odom at 30 Hz on `/zed/zed_node/odom` (BEST_EFFORT QoS)
- **`BridgeVerifier`** — subscribes to both MAVROS output topics and logs counts every 5 s

Use this to verify the bridge pipeline without real ZED hardware.
