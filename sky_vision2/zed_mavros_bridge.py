"""
ZED camera odometry to MAVROS bridge with automatic home setting.

ZED outputs in its own frame. ArduPilot expects NED (North-East-Down).
MAVROS vision_pose plugin expects ENU (East-North-Up) and converts to NED.

ZED odom frame has both X and Y axes inverted relative to ROS ENU (equivalent
to a 180° rotation around Z). position.x, position.y, twist.linear.x, and
twist.linear.y are negated, and the orientation quaternion is rotated 180° around Z
before publishing. MAVROS vision_pose plugin then converts ENU → NED automatically.

- Monitors EKF3 health via /mavros/estimator_status
- Calls /mavros/cmd/set_home once EKF has been healthy for STABLE_SECS

Required ArduPilot parameters:
    EK3_SRC1_POSXY = 6  (ExternalNav)
    EK3_SRC1_VELXY = 6  (ExternalNav)
    EK3_SRC1_POSZ  = 1  (Baro)
    EK3_SRC1_VELZ  = 0  (None)
    EK3_SRC1_YAW   = 6  (ExternalNav)
    VISO_TYPE      = 1
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import EstimatorStatus
from mavros_msgs.srv import CommandHome


STABLE_SECS = 5.0


class ZedMavrosBridge(Node):
    def __init__(self):
        super().__init__('zed_mavros_bridge')

        self.declare_parameter('zed_odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('mavros_vision_pose_topic', '/mavros/vision_pose/pose')
        self.declare_parameter('mavros_vision_speed_topic', '/mavros/vision_speed/speed_twist')

        zed_topic   = self.get_parameter('zed_odom_topic').get_parameter_value().string_value
        pose_topic  = self.get_parameter('mavros_vision_pose_topic').get_parameter_value().string_value
        speed_topic = self.get_parameter('mavros_vision_speed_topic').get_parameter_value().string_value

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._sub = self.create_subscription(Odometry, zed_topic, self._odom_cb, sensor_qos)
        self._ekf_sub = self.create_subscription(
            EstimatorStatus, '/mavros/estimator_status', self._ekf_cb, 10
        )

        self._pose_pub  = self.create_publisher(PoseStamped, pose_topic, 10)
        self._speed_pub = self.create_publisher(TwistStamped, speed_topic, 10)
        self._home_client = self.create_client(CommandHome, '/mavros/cmd/set_home')

        self._msg_count     = 0
        self._healthy_since = None
        self._home_set      = False

        self.get_logger().info(
            f'ZED-MAVROS bridge started\n'
            f'  ZED odom     : {zed_topic}\n'
            f'  Vision pose  : {pose_topic}\n'
            f'  Vision speed : {speed_topic}\n'
            f'  Frame: ZED ENU -> MAVROS NED (automatic)\n'
            f'  Home set after {STABLE_SECS}s of healthy EKF'
        )

    def _odom_cb(self, msg: Odometry):
        stamp = msg.header.stamp

        # 180° rotation around Z: negate X and Y, rotate quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = -msg.pose.pose.position.x
        pose_msg.pose.position.y = -msg.pose.pose.position.y
        pose_msg.pose.position.z =  msg.pose.pose.position.z
        # q_result = q_z180 * q_orig, q_z180 = (0, 0, 1, 0)
        pose_msg.pose.orientation.x = -qy
        pose_msg.pose.orientation.y =  qx
        pose_msg.pose.orientation.z =  qw
        pose_msg.pose.orientation.w = -qz
        self._pose_pub.publish(pose_msg)

        speed_msg = TwistStamped()
        speed_msg.header.stamp    = stamp
        speed_msg.header.frame_id = 'map'
        speed_msg.twist.linear.x  = -msg.twist.twist.linear.x
        speed_msg.twist.linear.y  = -msg.twist.twist.linear.y
        speed_msg.twist.linear.z  =  msg.twist.twist.linear.z
        self._speed_pub.publish(speed_msg)

        self._msg_count += 1
        if self._msg_count % 100 == 0:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.get_logger().info(
                f'[{self._msg_count}] ZED -> MAVROS | '
                f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
                f'quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})'
            )

    def _ekf_cb(self, msg: EstimatorStatus):
        if self._home_set:
            return

        now = self.get_clock().now()

        if msg.pos_horiz_rel:
            if self._healthy_since is None:
                self._healthy_since = now
                self.get_logger().info('EKF healthy — starting home countdown')

            elapsed = (now - self._healthy_since).nanoseconds / 1e9
            if elapsed >= STABLE_SECS:
                self._send_set_home()
        else:
            if self._healthy_since is not None:
                self.get_logger().info('EKF lost — resetting countdown')
            self._healthy_since = None

    def _send_set_home(self):
        if not self._home_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('set_home service not available, retrying')
            self._healthy_since = None
            return

        req = CommandHome.Request()
        req.current_gps = True
        future = self._home_client.call_async(req)
        future.add_done_callback(self._home_cb)

    def _home_cb(self, future):
        try:
            result = future.result()
            if result.success:
                self._home_set = True
                self.get_logger().info('HOME SET from vision EKF — ready to arm')
            else:
                self.get_logger().warning(f'set_home failed (result={result.result}), retrying')
                self._healthy_since = None
        except Exception as e:
            self.get_logger().error(f'set_home error: {e}')
            self._healthy_since = None


def main(args=None):
    rclpy.init(args=args)
    node = ZedMavrosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
