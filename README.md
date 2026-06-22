# sky_vision2

The **vision bridge package** for the SkyRats IMAV 2026 drone. It reads position data from the ZED camera and sends it to the Pixhawk flight controller so ArduPilot knows where the drone is.

## What this package does

The ZED camera continuously outputs its estimated position (odometry). ArduPilot needs that data in a specific format (NED coordinates via MAVROS). This package bridges the two:

```
ZED camera  →  /zed/zed_node/odom  →  zed_mavros_bridge  →  MAVROS  →  ArduPilot EKF3
```

It also:
- Converts the ZED coordinate frame to the NED frame ArduPilot expects
- Watches the EKF health status and automatically sets the home position once the EKF has been stable for 5 seconds

## Quick start

```bash
cd ~/sky_ws2
colcon build --packages-select sky_vision2
source install/setup.bash
export ROS_DOMAIN_ID=42

# Start everything (ZED + MAVROS + bridge)
ros2 launch sky_vision2 zed_mavros_fc.launch.py
```

Wait for: `HOME SET from vision EKF — ready to arm`

## Nodes

### `zed_mavros_bridge`

The main node. Runs continuously during flight.

**What it subscribes to:**

| Topic | What it is |
|-------|-----------|
| `/zed/zed_node/odom` | ZED camera position + velocity output |
| `/mavros/estimator_status` | ArduPilot EKF health — used to know when to set home |

**What it publishes:**

| Topic | What it is |
|-------|-----------|
| `/mavros/vision_pose/pose` | Drone position in NED → feeds ArduPilot EKF3 |
| `/mavros/vision_speed/speed_twist` | Drone velocity in NED → feeds ArduPilot EKF3 |

**Startup sequence:**

1. Node starts and subscribes to the ZED odom topic
2. Converts every ZED message to NED and publishes to MAVROS at ~30 Hz
3. Watches EKF health — when `pos_horiz_rel` stays `True` for 5 seconds, calls `set_home`
4. Logs `HOME SET from vision EKF — ready to arm` — drone is now ready

**Keep the drone still for the first ~20 seconds after launch** so the EKF converges and home is set at the correct position.

### `test_zed_odom`

A testing utility — no hardware needed. Publishes fake circular odometry on `/zed/zed_node/odom` at 30 Hz, and also listens to the MAVROS output topics to verify the bridge is working.

Use it when you want to test the bridge or SITL without a real ZED camera.

## Launch files

| File | Starts | When to use |
|------|--------|------------|
| `zed_mavros_fc.launch.py` | ZED + MAVROS + bridge | **Normal hardware flight** |
| `mavros_fc.launch.py` | MAVROS + bridge only | ZED is already running in another terminal |
| `zed.launch.py` | ZED only | Just checking the camera |
| `zed_mavros_sitl.launch.py` | ZED + MAVROS + bridge | SITL — see warning below |

All launch files set `ROS_DOMAIN_ID=42` automatically.

### Launch arguments

```bash
# Change the flight controller port (default: Jetson Telem2 UART)
ros2 launch sky_vision2 zed_mavros_fc.launch.py fcu_url:=/dev/ttyTHS1:921600

# Point at ArduPilot SITL instead of real hardware
ros2 launch sky_vision2 mavros_fc.launch.py fcu_url:=tcp://127.0.0.1:5760

# Different ZED camera model
ros2 launch sky_vision2 zed_mavros_fc.launch.py camera_model:=zed2
```

### SITL warning

`zed_mavros_sitl.launch.py` does not apply the FastDDS shared-memory fix. If MAVROS topics appear but carry no data, run this first:

```bash
rm -f /dev/shm/fastrtps_*
```

Or just use `mavros_fc.launch.py` with `fcu_url:=tcp://127.0.0.1:5760` — it has the fix built in.

## Testing without hardware

**Terminal 1:**
```bash
export ROS_DOMAIN_ID=42
ros2 run sky_vision2 zed_mavros_bridge
```

**Terminal 2:**
```bash
export ROS_DOMAIN_ID=42
ros2 run sky_vision2 test_zed_odom
```

**Terminal 3** — verify data is flowing:
```bash
export ROS_DOMAIN_ID=42
ros2 topic hz /mavros/vision_pose/pose        # should be ~30 Hz
ros2 topic hz /mavros/vision_speed/speed_twist # should be ~30 Hz
```

## Coordinate frame note

The ZED camera outputs X=North, Y=West, Z=Down. ArduPilot EKF3 expects NED (X=North, Y=East, Z=Down). The bridge fixes this by negating Y position, Y velocity, and the Y and Z parts of the rotation quaternion.

MAVROS does **not** do this conversion automatically when used with ArduPilot — the bridge must send NED directly.

## Config files

| File | Purpose |
|------|---------|
| `config/apm_pluginlists_vision.yaml` | Tells MAVROS which plugins to load (only the ones needed for visual odometry) |
| `config/fastdds_no_shm.xml` | Disables DDS shared memory — prevents stale data after MAVROS restarts |

## Important: do not run two bridges at once

Both `sky_vision2` and `indoor_2026` have a `zed_mavros_bridge` node. Running both simultaneously sends duplicate messages to MAVROS and confuses the EKF. Only one should be active at a time.

Check before launching:
```bash
ros2 node list | grep zed_mavros_bridge
# should show exactly one line
```
