"""
Standalone ZED camera launch.

Default camera model: zed2i

Run:
    ros2 launch sky_vision2 zed.launch.py

Override camera model:
    ros2 launch sky_vision2 zed.launch.py camera_model:=zedm

Verify camera is publishing:
    ros2 topic echo /zed/zed_node/odom
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model (zed, zed2, zed2i, zedm, zedx, zedxm)',
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'
            ])
        ),
        launch_arguments={'camera_model': LaunchConfiguration('camera_model')}.items(),
    )

    return LaunchDescription([
        camera_model_arg,
        zed_launch,
    ])
