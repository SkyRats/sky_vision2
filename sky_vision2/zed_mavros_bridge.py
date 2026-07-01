"""
ZED camera odometry to MAVROS bridge.

Position/orientation only — no vision_speed is published. The ZED ROS2 wrapper's
publishOdom() never fills the Odometry message's twist field, so msg.twist.twist.linear
is always exactly zero regardless of actual motion. Forwarding that as
VISION_SPEED_ESTIMATE would feed the EKF a false "velocity = 0" measurement instead of
just omitting velocity, which is worse than not sending it at all.

An optional yaw_offset_rad is applied to the orientation (quaternion multiply on the
left by a pure-Z rotation). Use this to zero out the ZED's initial heading:
  yaw_offset_rad = -(initial yaw reading in radians)

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
        self.declare_parameter('yaw_offset_rad', 0.0)

        zed_topic    = self.get_parameter('zed_odom_topic').get_parameter_value().string_value
        pose_topic   = self.get_parameter('mavros_vision_pose_topic').get_parameter_value().string_value
        yaw_offset   = self.get_parameter('yaw_offset_rad').get_parameter_value().double_value

        # Pre-compute yaw-offset correction quaternion (pure Z rotation)
        # q_corr = (0, 0, sin(offset/2), cos(offset/2))
        self._corr_z = math.sin(yaw_offset / 2.0)
        self._corr_w = math.cos(yaw_offset / 2.0)

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
            f'  ZED odom     : {zed_topic}\n'
            f'  Vision pose  : {pose_topic}\n'
            f'  Yaw offset   : {math.degrees(yaw_offset):.1f} deg\n'
            f'  (no vision_speed — ZED wrapper never populates twist)'
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

        # apply yaw offset: q_out = q_corr * q_in
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
