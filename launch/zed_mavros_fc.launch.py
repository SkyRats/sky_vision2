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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyTHS1:921600',
        description='MAVLink URL for MAVROS — Jetson Telem2 UART default: /dev/ttyTHS1:921600',
    )
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model (zed, zed2, zed2i, zedm, zedx, zedxm)',
    )
    zed_odom_topic_arg = DeclareLaunchArgument(
        'zed_odom_topic',
        default_value='/zed/zed_node/odom',
        description='ZED ROS2 wrapper odometry topic',
    )

    # Use a fixed domain ID to avoid DDS type conflicts with other ROS2 nodes
    # on the same network that may have different mavros_msgs builds.
    domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')

    # Disable Fast-DDS shared memory transport to prevent stale type-signature
    # conflicts from previous MAVROS runs persisting in /dev/shm.
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

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])
        ),
        launch_arguments={'camera_model': LaunchConfiguration('camera_model')}.items(),
    )

    return LaunchDescription([
        domain_id,
        no_shm,
        fcu_url_arg,
        camera_model_arg,
        zed_odom_topic_arg,
        zed_launch,
        mavros_node,
        bridge_node,
    ])
