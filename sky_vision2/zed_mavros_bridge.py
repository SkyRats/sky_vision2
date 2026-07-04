"""
ZED camera odometry to MAVROS bridge.

Position/orientation only — no vision_speed is published. The ZED ROS2 wrapper's
publishOdom() never fills the Odometry message's twist field, so msg.twist.twist.linear
is always exactly zero regardless of actual motion. Forwarding that as
VISION_SPEED_ESTIMATE would feed the EKF a false "velocity = 0" measurement instead of
just omitting velocity, which is worse than not sending it at all.

On the first odom message the bridge auto-zeros the initial NED yaw so ArduPilot
always sees yaw=0 at boot regardless of drone orientation. The correction is derived
from the MAVROS ENU→NED transform chain:

  q_ned = Rz(π/2)·Rx(π) * q_bridge * Rx(π)

For a pure-Z ENU input at angle φ the NED yaw comes out as a function of φ.
Setting the bridge correction offset = π/2 − φ_initial forces that NED yaw to 0.

ArduPilot parameters required:
    EK3_SRC1_POSXY = 6  (ExternalNav)
    EK3_SRC1_VELXY = 0  (None — no external velocity source is published)
    EK3_SRC1_POSZ  = 1  (Baro)
    EK3_SRC1_VELZ  = 0  (None)
    EK3_SRC1_YAW   = 6  (ExternalNav)
    VISO_TYPE      = 1
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class ZedMavrosBridge(Node):
    def __init__(self):
        super().__init__('zed_mavros_bridge')

        self.declare_parameter('zed_odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('mavros_vision_pose_topic', '/mavros/mavros/pose')

        zed_topic  = self.get_parameter('zed_odom_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('mavros_vision_pose_topic').get_parameter_value().string_value

        # Correction quaternion (pure Z); set on first odom message via _auto_zero_yaw().
        self._corr_z = 0.0
        self._corr_w = 1.0
        self._yaw_zeroed = False

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._sub       = self.create_subscription(Odometry, zed_topic, self._odom_cb, sensor_qos)
        self._pose_pub  = self.create_publisher(PoseStamped, pose_topic, 10)

        self._msg_count = 0

        self.get_logger().info(
            f'ZED-MAVROS bridge started\n'
            f'  ZED odom    : {zed_topic}\n'
            f'  Vision pose : {pose_topic}\n'
            f'  Yaw offset  : auto-zero on first odom message\n'
            f'  (no vision_speed — ZED wrapper never populates twist)'
        )

    def _auto_zero_yaw(self, qx, qy, qz, qw):
        # Extract the ZED's initial ENU yaw (rotation around Z).
        # Standard atan2 formula — returns full [-pi, pi] range.
        yaw_zed = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
        # MAVROS applies Rz(pi/2)*Rx(pi) * q * Rx(pi) when converting ENU→NED.
        # For a pure-Z ENU input at angle phi the NED yaw = 0 when phi = pi/2.
        # So set the bridge correction so the effective phi is always pi/2:
        #   offset = pi/2 - yaw_zed
        offset = math.pi / 2.0 - yaw_zed
        self._corr_z = math.sin(offset / 2.0)
        self._corr_w = math.cos(offset / 2.0)
        self._yaw_zeroed = True
        self.get_logger().info(
            f'Yaw auto-zeroed: ZED ENU yaw={math.degrees(yaw_zed):.1f}° '
            f'→ correction={math.degrees(offset):.1f}° '
            f'→ initial NED yaw=0°'
        )

    def _odom_cb(self, msg: Odometry):
        stamp = msg.header.stamp

        # ZED observed axes: X=East, Y=North, Z=Down
        # NED target:        X=North, Y=East, Z=Down
        # Position/velocity: swap X and Y (no negation needed)
        # Quaternion: pass through, then apply yaw offset only
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        if not self._yaw_zeroed:
            self._auto_zero_yaw(qx, qy, qz, qw)

        # apply yaw correction: q_out = q_corr * q_in
        cz, cw = self._corr_z, self._corr_w
        ox = cw * qx - cz * qy
        oy = cw * qy + cz * qx
        oz = cw * qz + cz * qw
        ow = cw * qw - cz * qz

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = -msg.pose.pose.position.y   # North
        pose_msg.pose.position.y =  msg.pose.pose.position.x   # East
        pose_msg.pose.position.z = msg.pose.pose.position.z
        pose_msg.pose.orientation.x = ox
        pose_msg.pose.orientation.y = oy
        pose_msg.pose.orientation.z = oz
        pose_msg.pose.orientation.w = ow
        self._pose_pub.publish(pose_msg)

        self._msg_count += 1
        if self._msg_count == 150:
            self.get_logger().info('Vision data flowing — ready to arm once EKF converges')
        if self._msg_count % 300 == 0:
            p = msg.pose.pose.position
            self.get_logger().info(
                f'[{self._msg_count}] pos=({p.x:.3f}, {-p.y:.3f}, {p.z:.3f})'
            )


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
