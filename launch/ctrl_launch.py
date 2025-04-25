from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    depth_profile_launch_arg = DeclareLaunchArgument(
            'depth_profile', default_value="640x480x30")
    enable_depth_launch_arg = DeclareLaunchArgument(
            'enable_depth', default_value="true")

    return LaunchDescription([
        depth_profile_launch_arg,
        enable_depth_launch_arg,
        Node(
            package='realsense2_camera',
            namespace='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[{
                'depth_module.profile': LaunchConfiguration('depth_profile'),
                'enable_depth': LaunchConfiguration('enable_depth')
            }]
        )#,
        #Node(
        #    package='controller',
        #    namespace='bot_controller',
        #    executable='depth_processing'
        #)#,
        #Node(
        #    package='controller',
        #    namespace='bot_controller',
        #    executable="hl_ctrl"
        #)
    ])

