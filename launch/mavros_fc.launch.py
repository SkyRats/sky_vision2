"""
Standalone MAVROS launch for a real ArduPilot FC.

Default: Jetson connected to Pixhawk 6C Telem2 via UART (/dev/ttyTHS1:921600)

Run:
    ros2 launch sky_vision2 mavros_fc.launch.py

Override port/baud:
    ros2 launch sky_vision2 mavros_fc.launch.py fcu_url:=/dev/ttyTHS0:921600

Verify connection:
    ros2 topic echo /mavros/state
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyTHS1:921600',
        description='MAVLink URL for MAVROS — Jetson Telem2 UART default: /dev/ttyTHS1:921600',
    )

    domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')

    fastdds_profile = PathJoinSubstitution([
        FindPackageShare('sky_vision2'), 'config', 'fastdds_no_shm.xml'
    ])
    no_shm = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile)

    pluginlists_yaml = PathJoinSubstitution([
        FindPackageShare('sky_vision2'), 'config', 'apm_pluginlists_vision.yaml'
    ])
    apm_config_yaml = PathJoinSubstitution([
        FindPackageShare('mavros'), 'launch', 'apm_config.yaml'
    ])

    zed_odom_topic_arg = DeclareLaunchArgument(
        'zed_odom_topic',
        default_value='/zed/zed_node/odom',
        description='ZED ROS2 wrapper odometry topic',
    )
    yaw_offset_arg = DeclareLaunchArgument(
        'yaw_offset_rad',
        default_value='-1.5708',
        description='Yaw offset (rad) applied after NED correction to zero initial heading',
    )

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
            'yaw_offset_rad': LaunchConfiguration('yaw_offset_rad'),
        }],
    )

    watchdog_node = Node(
        package='sky_vision2',
        executable='ekf_home_watchdog',
        name='ekf_home_watchdog',
        output='screen',
    )

    return LaunchDescription([
        domain_id,
        no_shm,
        fcu_url_arg,
        zed_odom_topic_arg,
        yaw_offset_arg,
        mavros_node,
        bridge_node,
        watchdog_node,
    ])
