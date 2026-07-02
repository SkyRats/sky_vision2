# EkfHomeWatchdog — Node Reference

## Node identity

- **Package:** `sky_vision2`
- **Executable:** `ekf_home_watchdog` (entry point: `sky_vision2.ekf_home_watchdog:main`)
- **Source:** `sky_vision2/ekf_home_watchdog.py`

## Why this node exists — fallback, not primary

**The team's actual, trusted mechanism is the onboard Lua script**, `indoor_2026/fc_scripts/ekf_set_home.lua`, which gates on EKF health/variance before calling `vehicle:set_home_to_current_location(false)`. The drone flies with it — that's the source of truth. It's just **not currently deployed on this specific Pixhawk 6C**, which has no SD card (ArduPilot loads Lua scripts from `APM/scripts/` on the SD card) — deploy it once one is installed.

`ekf_home_watchdog` is a ROS2-side fallback for whenever there's no SD card. Verified 2026-07-01: the MAVLink command it triggers (`/mavros/mavros/set_home` → `MAV_CMD_DO_SET_HOME`) and the Lua script's `vehicle:set_home_to_current_location()` both resolve to the identical ArduPilot function, `Copter::set_home_to_current_location()` in `ArduCopter/commands.cpp` — so this node is not a weaker substitute for the set_home call itself. It IS a weaker substitute for the *gating logic* around that call — see caveats below. **Not independently verified end-to-end**: a live test the same day got `MAV_RESULT_FAILED` (see bottom of this doc) — the team's real flights clearly do get home set somehow, this specific fallback path just hasn't been confirmed working on real hardware yet.

## Topics

| Direction | Topic | Type | QoS | Purpose |
|-----------|-------|------|-----|---------|
| Subscribes | `local_position_topic` param (default `/mavros/mavros/pose`) | `geometry_msgs/PoseStamped` | BEST_EFFORT, VOLATILE, depth=10 | Position-moved + stability gating |
| Subscribes | `/mavros/state` | `mavros_msgs/State` | default (RELIABLE) | `connected` flag |
| Subscribes | `/mavros/mavros/home` | `mavros_msgs/HomePosition` | default (RELIABLE) | Logged only, confirms the service call took effect |
| Service client | `/mavros/mavros/set_home` | `mavros_msgs/srv/CommandHome` | — | Called once, `current_gps=True` |

`command` and `home_position` are already in `config/apm_pluginlists_vision.yaml`'s allowlist — no MAVROS config change was needed to add this node.

### Topic caveat — verified live 2026-07-01

`local_position` plugin has the same relative-topic/namespace-collapse quirk as `vision_pose` (see `bridge_node.md`): its real topics are `/mavros/mavros/pose`, `/mavros/mavros/odom`, `/mavros/mavros/pose_cov`, etc — **not** `/mavros/local_position/pose` (an earlier version of this node used that wrong default; fixed after live testing showed the topic never appears).

Worse: on this hardware, `local_position`'s own outputs carry **zero live messages** — checked via `ros2 topic hz` against the running production stack, no data in 5+ seconds despite an active publisher endpoint. `LOCAL_POSITION_NED` belongs to ArduPilot's `STREAM_POSITION` group (`SR2_POSITION` on Telem2), which — like `SR2_EXTRA3` for `estimator_status` — appears to default to `0`. So in practice, `/mavros/mavros/pose` currently only carries the **bridge's own outgoing vision pose** (confirmed: it's the only publisher actually producing messages there). This means the gating check confirms vision data is being *sent*, not that the FC's EKF is actually *fusing* it — weaker than the Lua script's `ahrs:get_position()` check, which read the FC's own state. If `SR2_POSITION` is ever set >0, `local_position`'s publisher will start writing to this same topic name too (a pre-existing, unrelated collision) — re-verify before relying on this more strictly.

## Gating logic (mirrors `ekf_set_home.lua`)

| Lua | This node |
|-----|-----------|
| `ahrs:get_position()` moved >2cm off origin | `/mavros/mavros/pose` magnitude > `position_moved_thresh_m` (default 0.02) — in practice this is the bridge's own outgoing pose, see caveat above |
| `ahrs:healthy()` + `ahrs:get_variances()` under threshold | **Not equivalent.** `/mavros/estimator_status` never publishes (`SR2_EXTRA3=0` on Telem2 — see `.claude/rules/bridge_node.md`), and local_position covariance/pose isn't actually streaming either (see caveat above). Substitute: `/mavros/state.connected == True` and no position jump larger than `jump_thresh_m` (default 0.5m) between consecutive samples. **This cannot detect slow EKF drift**, only sudden discontinuities — a real capability gap versus the Lua script's variance check. |
| 5000ms continuous stability (`STABLE_REQUIRED_MS`) | `stable_required_ms` param, same default |
| `vehicle:set_home_to_current_location(false)` | `/mavros/mavros/set_home` with `current_gps=True` |

Fires **once and latches** (`_home_done`) — does not re-arm on a disarm/re-arm cycle, matching the Lua script's `home_done` flag.

## Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `local_position_topic` | `/mavros/mavros/pose` | Intended as ArduPilot's own reported position; in practice currently only carries the bridge's own outgoing vision pose (see topic caveat above) |
| `position_moved_thresh_m` | `0.02` | Confirms vision is actually fusing (not stuck at origin) |
| `stable_required_ms` | `5000` | How long the gate must hold before calling `set_home` |
| `jump_thresh_m` | `0.5` | Discontinuity larger than this resets the stability timer |
| `check_hz` | `4.0` | Gating loop rate |

## Retry behavior

On `set_home` failure (service returns `success=False`) or an exception, the node resets `_stable_since` so the next tick re-runs the full stability wait before retrying — no separate backoff timer, mirroring the Lua script's simple "reset and retry" behavior.

## Verification

```bash
ros2 topic echo /mavros/mavros/home --once   # before: zero position (or no message at all)
# ... wait for "HOME SET from vision EKF — ready to arm" in the node's log ...
ros2 topic echo /mavros/mavros/home --once   # after: non-zero, matches /mavros/mavros/pose
```

## Known-failing live test, 2026-07-01 — root cause and open question

Calling the service directly against the running production stack (`ros2 service call /mavros/mavros/set_home mavros_msgs/srv/CommandHome "{current_gps: true}"`) returned `success=False, result=4` (`MAV_RESULT_FAILED`).

Traced in ArduPilot source: `set_home_to_current_location()` (`ArduCopter/commands.cpp:40`) requires `ahrs.get_location()` to succeed, and the inner `set_home()` (`commands.cpp:58`) explicitly checks `ahrs.get_origin()` first and fails immediately if it's unset. **EKF origin and home are different things** — origin is the geodetic reference point used to convert local NED position to a global `Location`; home is what `set_home` sets. Origin only auto-sets via GPS lock (`AP_NavEKF3_Measurements.cpp:687`, requires `gpsGoodToAlign`) or a range beacon (`:1000`) — confirmed by reading `readyToUseExtNav()` (`AP_NavEKF3_Control.cpp:614`) that ExternalNav/vision fusion does **not** set origin as a side effect. Live-checked `/mavros/mavros/raw/fix` during the failing test: zero GPS messages.

**This is not necessarily a real problem** — the team's actual flights work, so origin evidently gets established somehow (a GPS fix, a manual GCS "set origin here" step, or something not yet identified) in normal operation. It just didn't happen in that specific test session. If this fallback node is ever actually relied on for a real flight, confirm origin gets set before trusting it — e.g. watch for an `EKF3: origin already set` or equivalent GCS message, or check `/mavros/mavros/home` actually goes non-zero after this node's "HOME SET" log line.
