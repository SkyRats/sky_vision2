# Yaw and Frame Convention — ZED → MAVROS → ArduPilot EKF3

> **Superseded, 2026-07-01:** this doc describes an older bridge design (MAVROS auto ENU→NED via the `vision_pose_estimate` plugin, +π/2 quaternion offset). The bridge running today (`zed_mavros_bridge.py`) does its own axis remap in `_odom_cb` and does not depend on this conversion path — see `.claude/rules/bridge_node.md` for the current, verified behavior. Keeping this file for the MAVROS/ArduPilot source-level background research (Eigen yaw clamping, EKF3 fusion chain), which is still generally useful, but don't treat the "NED alignment offset" section below as current.
>
> **Fix available, 2026-07-03:** the Eigen `eulerAngles(2,1,0)` yaw-clamping bug described below (see "Can you send −π, 0, +π from ZED?") now has a real fix — `git@github.com:odraudE31/mavros.git`, branch `fix/vision-pose-yaw-clamping`, replaces the Eigen decomposition in `quaternion_to_rpy` with an atan2-based ZYX decomposition. Built as an opt-in overlay in `~/sky_ws2` (apt MAVROS untouched by default) — see `docs/mavros_patched.md` at the workspace root. This means the "south/west yaw folds to π" caveat no longer applies when the overlay is active.

## Full pipeline at a glance

```
ZED odom (ENU / FLU)
    │
    │  pose_msg.pose = msg.pose.pose   (quaternion passed verbatim)
    ▼
MAVROS /mavros/vision_pose/pose
    │
    │  vision_pose_estimate plugin applies two chained rotations to the quaternion:
    │    1. BASELINK_TO_AIRCRAFT → Rx(π): FLU → FRD (body frame)
    │    2. ENU_TO_NED           → Rz(π/2)·Rx(π): world ENU → world NED
    │  Position reflection: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
    │
    ▼
MAVLink VISION_POSITION_ESTIMATE  (NED / FRD)
    │
    │  MAVROS extracts Euler yaw from the NED quaternion with Eigen eulerAngles(2,1,0)
    │  and fills: vp.roll, vp.pitch, vp.yaw
    │
    ▼
ArduPilot GCS_Common → AP_VisualOdom.from_euler(roll, pitch, yaw) → EKF3 fusion
```

---

## What "front" means in each frame

| Frame | Convention | Body front axis | World X axis |
|-------|------------|-----------------|--------------|
| ZED odom (world) | ENU | — | East |
| ZED body | FLU | +X = Forward (camera lens direction) | — |
| MAVROS world | ENU | — | East |
| ArduPilot world | NED | — | North (see caveat below) |
| ArduPilot body | FRD | +X = Forward (nose) | — |

**"Front" is always the body +X axis.** Yaw does not define "front" — it measures the angle of that front axis relative to the world reference direction.

### What yaw=0 actually means

**Outdoor with GPS:** NED "North" = magnetic/geographic North. Yaw=0 → nose pointing North.

**Indoor without GPS (your setup):** There is no geographic North. ArduPilot's EKF3 "North" is initialized from whichever source `EK3_SRC1_YAW` points to:
- If a compass is fused: "North" = magnetic North, same as outdoor.
- If `EK3_SRC1_YAW=6` (ExternalNav only): "North" = whatever heading the ZED reported at EKF initialization. The ZED's odom frame origin is wherever the camera was pointing when it booted. **There is no fixed absolute reference.**

In practice for indoor: yaw=0 is arbitrary and depends on where the drone was pointing at startup. What matters operationally is **yaw consistency** — the EKF tracks relative heading correctly as long as the source is stable, regardless of what "zero" is.

### Yaw angle sign convention (NED, FRD body)

In NED, yaw is measured **clockwise from North** when viewed from above:
- Yaw = 0 → nose points toward EKF "North"
- Yaw = π/2 → nose points toward EKF "East"
- Yaw = π (or −π) → nose points "South" (opposite of North)
- Yaw = −π/2 → nose points toward EKF "West"

ENU ↔ NED yaw mapping (MAVROS conversion):
- ENU yaw = 0 (East) → NED yaw = π/2
- ENU yaw = π/2 (North) → NED yaw = 0

---

## Can you send −π, 0, +π from ZED?

**Yes — the quaternion representation is safe for all headings, including the ±π boundary.**

ZED publishes a quaternion, not raw Euler angles. Quaternions have no singularity at ±π yaw. The full conversion chain operates on quaternions internally:

```cpp
// MAVROS vision_pose_estimate.cpp
auto rpy = ftf::quaternion_to_rpy(
    ftf::transform_orientation_enu_ned(
        ftf::transform_orientation_baselink_aircraft(q)));
vp.yaw = rpy.z();
```

The Euler extraction step (`Eigen::eulerAngles(2,1,0)`) has a known constraint: **it clamps the yaw angle to [0, π]**. For NED headings in (π, 2π) (i.e., "south-to-west half"), Eigen returns an alternative valid Euler decomposition where roll=π, pitch=π and yaw∈[0,π] — which looks odd but is mathematically equivalent:

```
Rz(yaw') · Ry(π) · Rx(π)  =  Rz(yaw' − π)   [same rotation]
```

ArduPilot reconstructs the correct attitude quaternion with `from_euler(roll, pitch, yaw)`, so the EKF3 receives the right orientation regardless. The rotation is always transmitted correctly.

**The only visible symptom** is that GCS telemetry / logs will show roll≈π, pitch≈π, yaw≈some_small_value instead of roll≈0, pitch≈0, yaw≈−something. The EKF does not care.

See [MAVROS issue #444](https://github.com/mavlink/mavros/issues/444) for the history of this Eigen limitation.

---

## How ArduPilot fuses it into EKF3

Processing chain (all in ArduPilot source):

1. **GCS_Common.cpp** — decodes `VISION_POSITION_ESTIMATE`, calls:
   ```cpp
   AP::visodom().handle_pose_estimate(
       packet.usec, time_ms,
       packet.x, packet.y, packet.z,
       packet.roll, packet.pitch, packet.yaw, ...);
   ```

2. **AP_VisualOdom.cpp** — converts Euler → quaternion:
   ```cpp
   Quaternion attitude;
   attitude.from_euler(roll, pitch, yaw);   // yaw field consumed here
   _driver->handle_pose_estimate(..., attitude, ...);
   ```

3. **AP_VisualOdom_MAV.cpp** — sends to AHRS:
   ```cpp
   AP::ahrs().writeExtNavData(pos, attitude, posErr, angErr, ...);
   ```

4. **EKF3** fuses the external nav position **and** attitude (including yaw) as a separate observation. With `EK3_SRC1_YAW=6` (ExternalNav), the EKF trusts vision yaw over compass/IMU-derived heading. ArduPilot does **not** derive yaw from a position sequence — the `yaw` Euler field is consumed directly.

---

## NED alignment offset applied in the bridge

Without correction, ZED boots with identity quaternion (ENU yaw=0), which MAVROS converts to NED yaw=π/2 — the drone nose appears to point NED East instead of NED North. This makes position setpoints unintuitive (NED X+ moves the drone sideways, not forward).

**Fix implemented in `zed_mavros_bridge.py`:** every ZED pose is left-multiplied by a +π/2-around-Z quaternion before forwarding to MAVROS.

```
q_offset = (w=√2/2, x=0, y=0, z=√2/2)   [+π/2 around Z in ENU]
q_sent   = q_offset * q_zed
p_sent   = R(+π/2) * p_zed  →  (x, y, z) → (−y, x, z)
```

Why +π/2 (not −π/2): the ENU→NED yaw relationship is `yaw_ned = π/2 − yaw_enu`. To get NED yaw=0 at startup, we need ENU yaw=π/2. Since ZED boots at ENU yaw=0, we add +π/2 by left-multiplying with q_offset. A −π/2 offset would yield NED yaw=π (South) — wrong direction.

**After the fix:**

| Situation | ZED ENU yaw | Corrected ENU yaw | NED yaw | Meaning |
|-----------|------------|-------------------|---------|---------|
| Boot (drone stationary) | 0 | π/2 | 0 | Nose = NED North |
| Turned 90° right | −π/2 | 0 | π/2 | Nose = NED East |
| Turned 90° left | +π/2 | π | −π/2 (π) | Nose = NED West |
| Turned 180° | ±π | ±π/2 → | π | Nose = NED South |

The `frame_id='map'` header is irrelevant — MAVROS ignores it when `tf/listen=false` (the default).

---

## Quick-reference: yaw values at cardinal headings

Values **after** the NED alignment offset is applied in the bridge (+π/2 around Z).
"Boot heading" = whatever physical direction the drone nose faced when ZED initialized.

| Drone nose direction (physical) | ZED ENU yaw | After +π/2 offset (ENU) | NED yaw ArduPilot sees |
|---------------------------------|------------|--------------------------|------------------------|
| Boot heading (= NED North)      | 0          | π/2                      | 0                      |
| 90° right of boot               | −π/2       | 0                        | π/2  (East)            |
| 90° left of boot                | +π/2       | π                        | −π/2 (West)            |
| 180° from boot                  | ±π         | ±π/2 (compensated)       | π    (South)           |

Without the offset (old behavior):

| Drone nose direction | ZED ENU yaw | NED yaw ArduPilot saw |
|----------------------|-------------|----------------------|
| Boot heading         | 0           | π/2 (East — wrong)   |
| Turned 90° right     | −π/2        | π   (South — wrong)  |

---

## Sources

- [MAVROS vision_pose_estimate.cpp (ros2 branch)](https://github.com/mavlink/mavros/blob/ros2/mavros_extras/src/plugins/vision_pose_estimate.cpp)
- [MAVROS frame_tf.hpp](https://github.com/mavlink/mavros/blob/ros2/mavros/include/mavros/frame_tf.hpp)
- [MAVROS ftf_frame_conversions.cpp](https://github.com/mavlink/mavros/blob/ros2/mavros/src/lib/ftf_frame_conversions.cpp)
- [MAVROS issue #444 — eulerAngles [0,π] clamping](https://github.com/mavlink/mavros/issues/444)
- `git@github.com:odraudE31/mavros.git`, branch `fix/vision-pose-yaw-clamping` (commit `d61db77e`) — atan2-based fix for the above, see `docs/mavros_patched.md`
- [ArduPilot AP_VisualOdom.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_VisualOdom/AP_VisualOdom.cpp)
- [ArduPilot AP_VisualOdom_MAV.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp)
- [ArduPilot GCS_Common.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp)
