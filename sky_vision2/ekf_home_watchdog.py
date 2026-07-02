"""
ekf_home_watchdog.py

ROS2-side replacement for fc_scripts/ekf_set_home.lua (indoor_2026), which cannot
run because the Pixhawk 6C has no SD card (ArduPilot Lua scripts load from
APM/scripts/ on the SD card).

Mirrors the Lua script's gating logic as closely as the available ROS2/MAVROS
signals allow:

  Lua                                   | ROS2 equivalent here
  --------------------------------------|----------------------------------------
  ahrs:get_position() moved off origin  | /mavros/mavros/pose deviates from (0,0,0)
                                         | by > position_moved_thresh_m
  ahrs:healthy() + get_variances()      | NOT equivalent -- /mavros/estimator_status
                                         | never publishes (SR2_EXTRA3=0 on Telem2).
                                         | Substitute: /mavros/state.connected == True
                                         | AND no large jump between consecutive
                                         | position samples over the stability window.
                                         | This catches sudden discontinuities but
                                         | cannot detect slow EKF drift the way a real
                                         | variance report would.
  5000 ms continuous stability          | stable_required_ms param, same default
  vehicle:set_home_to_current_location  | /mavros/mavros/set_home, current_gps=True

IMPORTANT topic caveat (found via live testing 2026-07-01): MAVROS's local_position
plugin has the same relative-topic/namespace collapse as vision_pose (documented in
bridge_node.md) -- its real topics are /mavros/mavros/pose, /mavros/mavros/odom,
/mavros/mavros/pose_cov, etc, NOT /mavros/local_position/pose. Worse: on this
hardware, none of those local_position outputs actually carry live data --
LOCAL_POSITION_NED belongs to ArduPilot's STREAM_POSITION group (SR2_POSITION on
Telem2), which -- like SR2_EXTRA3 for estimator_status -- appears to default to 0.
So this node instead watches /mavros/mavros/pose, which in practice only carries
the BRIDGE's own outgoing vision pose (confirmed the only publisher actually
producing messages on that topic name). This is a weaker check than intended: it
confirms we're SENDING vision data, not that the FC's EKF is actually consuming or
fusing it. If SR2_POSITION is ever set >0 on the FC, local_position's own publisher
would start also writing to this same topic name (a pre-existing, unrelated topic
name collision) -- re-verify before relying on it more strictly.

Fires once and latches (mirrors the Lua script's home_done flag) -- does not
re-arm on disarm/re-arm cycles.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandHome


class EkfHomeWatchdog(Node):
    def __init__(self):
        super().__init__('ekf_home_watchdog')

        self.declare_parameter('local_position_topic', '/mavros/mavros/pose')
        self.declare_parameter('position_moved_thresh_m', 0.02)   # Lua: 2 cm
        self.declare_parameter('stable_required_ms', 5000)        # Lua: STABLE_REQUIRED_MS
        self.declare_parameter('jump_thresh_m', 0.5)              # discontinuity = "not stable"
        self.declare_parameter('check_hz', 4.0)                   # Lua: CHECK_HZ

        self._prev_pos = None
        self._pos_ever_moved = False
        self._stable_since = None
        self._home_done = False
        self._fcu_connected = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )
        topic = self.get_parameter('local_position_topic').get_parameter_value().string_value
        self._pos_sub = self.create_subscription(PoseStamped, topic, self._pos_cb, qos)
        self._state_sub = self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        # Logged only, not used for gating -- lets operators visually confirm
        # home actually changed from zero once the service call succeeds.
        self._home_sub = self.create_subscription(
            HomePosition, '/mavros/mavros/home', self._home_cb, 10)

        self._set_home_cli = self.create_client(CommandHome, '/mavros/mavros/set_home')

        check_hz = self.get_parameter('check_hz').get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / check_hz, self._tick)

        self.get_logger().info('ekf_home_watchdog started -- waiting for vision + EKF stability')

    # -- Subscriptions --------------------------------------------------------
    def _state_cb(self, msg: State):
        self._fcu_connected = msg.connected

    def _home_cb(self, msg: HomePosition):
        if self._home_done:
            self.get_logger().info(
                f'home_position now: ({msg.position.x:.2f}, {msg.position.y:.2f}, '
                f'{msg.position.z:.2f})'
            )

    def _pos_cb(self, msg: PoseStamped):
        p = msg.pose.position
        moved_thresh = self.get_parameter('position_moved_thresh_m').get_parameter_value().double_value
        jump_thresh = self.get_parameter('jump_thresh_m').get_parameter_value().double_value

        if not self._pos_ever_moved:
            if math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2) > moved_thresh:
                self._pos_ever_moved = True
                self.get_logger().info('vision active -- EKF position is moving')

        if self._prev_pos is not None:
            d = math.sqrt(
                (p.x - self._prev_pos[0]) ** 2 +
                (p.y - self._prev_pos[1]) ** 2 +
                (p.z - self._prev_pos[2]) ** 2
            )
            if d > jump_thresh:
                # Large discontinuity -- treat as "not stable", reset timer.
                # Substitute for ahrs get_variances()/healthy() gating we can't do.
                self._stable_since = None
        self._prev_pos = (p.x, p.y, p.z)

    # -- Main gating loop (mirrors Lua update()) -------------------------------
    def _tick(self):
        if self._home_done:
            return

        if not (self._pos_ever_moved and self._fcu_connected):
            self._stable_since = None
            return

        now = self.get_clock().now()
        if self._stable_since is None:
            self._stable_since = now
            self.get_logger().info('EKF stable -- starting stability timer')
            return

        elapsed_ms = (now - self._stable_since).nanoseconds / 1e6
        required_ms = self.get_parameter('stable_required_ms').get_parameter_value().integer_value
        if elapsed_ms < required_ms:
            return

        self._call_set_home()

    # -- set_home service call -------------------------------------------------
    def _call_set_home(self):
        if not self._set_home_cli.service_is_ready():
            self.get_logger().warning('set_home service not ready yet -- will retry')
            return
        req = CommandHome.Request()
        req.current_gps = True
        future = self._set_home_cli.call_async(req)
        future.add_done_callback(self._set_home_done_cb)

    def _set_home_done_cb(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'set_home service call exception: {e}')
            self._stable_since = None  # re-arm stability wait before retrying
            return
        if result.success:
            self._home_done = True
            self.get_logger().info('HOME SET from vision EKF -- ready to arm')
        else:
            self.get_logger().warning(f'set_home failed (result={result.result}) -- retrying')
            self._stable_since = None


def main(args=None):
    rclpy.init(args=args)
    node = EkfHomeWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
