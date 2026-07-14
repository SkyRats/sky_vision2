"""
MAVROS + ZED bridge launched with mavp2p multiplexing the FC serial link.

mavp2p fans the single physical FC connection into two UDP server endpoints:
  14551 → MAVROS (vision input + state feedback)
  14552 → sky_navigation Drone class (all MAVLink movement commands)

Run (hardware, USB ACM):
    ros2 launch sky_vision2 mavros_mavp2p_fc.launch.py

Run (hardware, UART Telem2):
    ros2 launch sky_vision2 mavros_mavp2p_fc.launch.py \
        mavp2p_source:=serial:/dev/ttyTHS1:921600

Run (SITL — mavp2p → SITL TCP):
    ros2 launch sky_vision2 mavros_mavp2p_fc.launch.py \
        mavp2p_source:=tcpc://127.0.0.1:5760

Prerequisites:
    mavp2p binary on PATH — https://github.com/bluenviron/mavp2p/releases
    (grab the linux_arm64 build for Jetson)

Verify:
    ros2 topic echo /mavros/state --once         # connected: True
    ros2 topic hz /mavros/vision_pose/pose       # ~30 Hz after ZED up
    ros2 topic hz /mavros/local_position/pose    # ~10 Hz after EKF converges
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mavp2p_source_arg = DeclareLaunchArgument(
        'mavp2p_source',
        default_value='serial:/dev/ttyACM0:921600',
        description=(
            'mavp2p source endpoint. '
            'serial:/dev/ttyACM0:921600  for USB, '
            'serial:/dev/ttyTHS1:921600  for Jetson UART, '
            'tcpc://127.0.0.1:5760       for ArduPilot SITL'
        ),
    )
    zed_odom_topic_arg = DeclareLaunchArgument(
        'zed_odom_topic',
        default_value='/zed/zed_node/odom',
        description='ZED ROS2 wrapper odometry topic',
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

    # mavp2p: owns the single FC link, fans out to two UDP server endpoints.
    # MAVROS dials 14551; sky_navigation Drone dials 14552.
    # Both must heartbeat so mavp2p can route replies back to each client.
    mavp2p_process = ExecuteProcess(
        cmd=[
            'mavp2p',
            LaunchConfiguration('mavp2p_source'),
            'udps:127.0.0.1:14551',
            'udps:127.0.0.1:14552',
        ],
        output='screen',
    )

    # MAVROS → mavp2p endpoint 14551; vision input and state feedback only.
    # The setpoint_position / setpoint_velocity plugins are NOT in the allowlist
    # so MAVROS cannot accidentally accept movement commands from ROS topics.
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        parameters=[
            pluginlists_yaml,
            apm_config_yaml,
            {
                'fcu_url': 'udp://127.0.0.1:0@127.0.0.1:14551',
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
        domain_id,
        no_shm,
        mavp2p_source_arg,
        zed_odom_topic_arg,
        mavp2p_process,
        mavros_node,
        bridge_node,
    ])
