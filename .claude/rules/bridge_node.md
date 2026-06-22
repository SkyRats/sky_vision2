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
| Publishes | `mavros_vision_pose_topic` param (default `/mavros/vision_pose/pose`) | `geometry_msgs/PoseStamped` | RELIABLE (default) |
| Publishes | `mavros_vision_speed_topic` param (default `/mavros/vision_speed/speed_twist`) | `geometry_msgs/TwistStamped` | RELIABLE (default) |
| Service client | `/mavros/cmd/set_home` | `mavros_msgs/srv/CommandHome` | async |

**Critical:** ZED driver publishes odom with BEST_EFFORT. The bridge subscription must use BEST_EFFORT — using RELIABLE silently receives nothing.

## Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `zed_odom_topic` | `/zed/zed_node/odom` | Change if using a non-standard ZED namespace |
| `mavros_vision_pose_topic` | `/mavros/vision_pose/pose` | Feeds EKF3 position |
| `mavros_vision_speed_topic` | `/mavros/vision_speed/speed_twist` | Feeds EKF3 velocity |

## CRITICAL: MAVROS does NOT convert ENU→NED for ArduPilot

When using MAVROS with `apm.launch` (ArduPilot), the `vision_pose` plugin passes `PoseStamped` data **directly** as `VISION_POSITION_ESTIMATE` without any ENU→NED conversion. ArduPilot EKF3 expects NED (X=North, Y=East, Z=Down).

**The bridge must publish NED, not ENU.** Do not assume MAVROS will handle the conversion.

## Frame transformation

ZED odom frame in the SkyRats mounting configuration:

| Axis | ZED direction | NED target | Correction |
|------|--------------|------------|-----------|
| X | North | North | none |
| Y | West | East | negate Y |
| Z | Down | Down | none |

The bridge should negate only Y. See `~/imav_2026_ws/.claude/rules/coordinate_frames.md` for the verified NED correction math and quaternion treatment.

**Note:** The current code in `zed_mavros_bridge.py` negates both X and Y (180° Z rotation). This predates confirmation that MAVROS with APM does not auto-convert ENU→NED. Review and test any frame correction changes against actual flight behavior before committing.

## EKF watchdog and auto-home

The node monitors `/mavros/estimator_status`. Once `pos_horiz_rel=True` holds for `STABLE_SECS=5.0` seconds, it calls `set_home` asynchronously. Log output:

```
EKF healthy — starting home countdown
HOME SET from vision EKF — ready to arm
```

Keep the drone **stationary** for the first ~20 s after launch so the EKF converges and the home point is set from the correct position.

## Bridge exclusivity

Never run `ZedMavrosBridge` and `pose_relay` (indoor_2026 in imav_2026_ws) simultaneously — both publish to `/mavros/vision_pose/pose` and will corrupt the EKF. Check before launching:

```bash
ros2 node list | grep -E "pose_relay|zed_mavros_bridge"
# must show 0 or 1, never both
```

## test_zed_odom utility

`sky_vision2/test_zed_odom.py` contains two nodes run in one process:
- **`ZedOdomPublisher`** — publishes synthetic circular odom at 30 Hz on `/zed/zed_node/odom` (BEST_EFFORT QoS)
- **`BridgeVerifier`** — subscribes to both MAVROS output topics and logs counts every 5 s

Use this to verify the bridge pipeline without real ZED hardware.
