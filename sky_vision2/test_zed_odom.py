"""
Test node for the ZED-MAVROS odometry bridge.

Publishes synthetic ZED odometry so the bridge can be exercised without
real hardware. Verifies that pose messages arrive on the bridge's actual
output topic (no vision_speed — the bridge doesn't publish it; see
zed_mavros_bridge.py's module docstring).

Usage (3 terminals):
    # Terminal 1 — run MAVROS (or just check topics without it):
    ros2 run sky_vision2 zed_mavros_bridge

    # Terminal 2 — run this test publisher:
    ros2 run sky_vision2 test_zed_odom

    # Terminal 3 — verify output:
    ros2 topic echo /mavros/vision_pose/pose
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseStamped,
    Point,
    Quaternion,
    Vector3,
)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


def _yaw_to_quaternion(yaw_rad):
    """Convert a yaw angle (radians) to a ROS quaternion (z-up, ENU)."""
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)


class ZedOdomPublisher(Node):
    """Publishes a synthetic circular trajectory as ZED odometry."""

    PUBLISH_HZ = 30.0
    RADIUS = 2.0      # circle radius in meters
    OMEGA = 0.2       # angular velocity in rad/s

    def __init__(self):
        super().__init__('zed_odom_test_publisher')

        self.declare_parameter('zed_odom_topic', '/zed/zed_node/odom')
        zed_topic = self.get_parameter('zed_odom_topic').get_parameter_value().string_value

        self._pub = self.create_publisher(Odometry, zed_topic, 10)
        self._timer = self.create_timer(1.0 / self.PUBLISH_HZ, self._publish)
        self._t = 0.0
        self.get_logger().info(f'Publishing synthetic ZED odom on {zed_topic}')

    def _publish(self):
        self._t += 1.0 / self.PUBLISH_HZ

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Circular trajectory in XY plane
        angle = self.OMEGA * self._t
        x = self.RADIUS * math.cos(angle)
        y = self.RADIUS * math.sin(angle)
        z = 1.0  # fixed altitude for repeatable SITL testing

        msg.pose.pose.position = Point(x=x, y=y, z=z)
        msg.pose.pose.orientation = _yaw_to_quaternion(angle + math.pi / 2)

        # Velocity: tangent to circle
        vx = -self.RADIUS * self.OMEGA * math.sin(angle)
        vy = self.RADIUS * self.OMEGA * math.cos(angle)
        vz = 0.5

        msg.twist.twist.linear = Vector3(x=vx, y=vy, z=vz)

        self._pub.publish(msg)


class BridgeVerifier(Node):
    """Subscribes to the bridge's actual output topic and logs what arrives."""

    def __init__(self):
        super().__init__('zed_bridge_verifier')

        self._pose_count = 0

        self._pose_sub = self.create_subscription(
            PoseStamped, '/mavros/vision_pose/pose', self._pose_cb, 10
        )
        self._check_timer = self.create_timer(5.0, self._report)
        self.get_logger().info('Verifier listening on /mavros/vision_pose/pose')

    def _pose_cb(self, msg: PoseStamped):
        self._pose_count += 1
        if self._pose_count % 30 == 0:
            p = msg.pose.position
            self.get_logger().info(
                f'vision_pose [{self._pose_count}]: '
                f'x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}'
            )

    def _report(self):
        if self._pose_count == 0:
            self.get_logger().warning(
                'No pose messages received yet — is zed_mavros_bridge running?'
            )
        else:
            self.get_logger().info(f'5s summary: pose_msgs={self._pose_count}')


def main(args=None):
    rclpy.init(args=args)

    publisher = ZedOdomPublisher()
    verifier = BridgeVerifier()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(verifier)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        publisher.destroy_node()
        verifier.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
