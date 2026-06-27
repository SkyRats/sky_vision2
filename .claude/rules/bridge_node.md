# ZedMavrosBridge — Node Reference

## Node identity

- **Package:** `sky_vision2`
- **Executable:** `zed_mavros_bridge` (entry point: `sky_vision2.zed_mavros_bridge:main`)
- **Source:** `sky_vision2/zed_mavros_bridge.py`

## Topics

| Direction | Topic | Type | QoS |
|-----------|-------|------|-----|
| Subscribes | `zed_odom_topic` param (default `/zed/zed_node/odom`) | `nav_msgs/Odometry` | **BEST_EFFORT**, VOLATILE, depth=10 |
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

## Frame convention

ZED odom follows ROS REP-105 (ENU). MAVROS `vision_pose_estimate` plugin converts ENU→NED automatically before sending `VISION_POSITION_ESTIMATE` to ArduPilot. **The bridge does not perform any manual frame correction.**

### NED alignment offset

ZED boots with identity orientation (ENU yaw=0). MAVROS converts this to NED yaw=π/2, meaning ArduPilot would see the drone nose pointing East at startup. The bridge fixes this by left-multiplying every pose by a +π/2-around-Z quaternion before publishing:

```python
q_offset = (w=√2/2, x=0, y=0, z=√2/2)   # +π/2 around Z
q_sent   = q_offset * q_zed
# Position: (x, y, z) → (−y, x, z)
```

Result: boot-time nose direction = NED North (yaw=0). All yaw deltas track correctly.

See `.claude/rules/yaw_frame_research.md` for the full derivation, MAVROS source references, the ±π Eigen clamping behavior, and ArduPilot's EKF3 fusion chain.

## EKF watchdog and auto-home

Monitors `/mavros/estimator_status.pos_horiz_rel`. Once `True` for `STABLE_SECS=5.0` continuous seconds → calls `set_home` async.

Log sequence:
```
EKF healthy — starting home countdown
HOME SET from vision EKF — ready to arm
```

**Keep drone stationary for first ~20 s after launch** so EKF converges and home is set at the correct position.

## Bridge exclusivity

`sky_vision2` is the only package that should run `zed_mavros_bridge`. Never run two instances simultaneously — both would publish to `/mavros/vision_pose/pose` and corrupt the EKF.

```bash
ros2 node list | grep zed_mavros_bridge   # must show exactly one
```

## test_zed_odom utility

`sky_vision2/test_zed_odom.py` runs two nodes in one process:
- **`ZedOdomPublisher`** — synthetic circular odom at 30 Hz on `/zed/zed_node/odom` (BEST_EFFORT)
- **`BridgeVerifier`** — subscribes to MAVROS output topics, logs counts every 5 s
