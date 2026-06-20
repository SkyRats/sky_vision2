"""
Launch file: ZED odometry bridge + MAVROS connected to a real ArduPilot FC.

Default: Jetson companion computer connected to Pixhawk 6C Telem2 via UART
    fcu_url = /dev/ttyTHS1:921600

Run on the Jetson:
    ros2 launch sky_vision2 zed_mavros_fc.launch.py

To use a different port or baud rate:
    ros2 launch sky_vision2 zed_mavros_fc.launch.py fcu_url:=/dev/ttyTHS0:921600

To test without real ZED hardware, in a second terminal run:
    ros2 run sky_vision2 test_zed_odom

Verify ArduPilot is receiving vision data:
    ros2 topic echo /mavros/vision_pose/pose
    ros2 topic echo /mavros/local_position/odom
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyTHS1:921600',
        description='MAVLink URL for MAVROS — Jetson Telem2 UART default: /dev/ttyTHS1:921600',
    )
    zed_odom_topic_arg = DeclareLaunchArgument(
        'zed_odom_topic',
        default_value='/zed/zed_node/odom',
        description='ZED ROS2 wrapper odometry topic',
    )

    pluginlists_yaml = PathJoinSubstitution([
        FindPackageShare('sky_vision2'), 'config', 'apm_pluginlists_vision.yaml'
    ])
    apm_config_yaml = PathJoinSubstitution([
        FindPackageShare('mavros'), 'launch', 'apm_config.yaml'
    ])

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            pluginlists_yaml,
            apm_config_yaml,
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
            },
        ],
    )

    bridge_node = Node(
        package='sky_vision2',
        executable='zed_mavros_bridge',
        name='zed_mavros_bridge',
        output='screen',
        parameters=[{
            'zed_odom_topic': LaunchConfiguration('zed_odom_topic'),
        }],
    )

    return LaunchDescription([
        fcu_url_arg,
        zed_odom_topic_arg,
        mavros_node,
        bridge_node,
    ])
