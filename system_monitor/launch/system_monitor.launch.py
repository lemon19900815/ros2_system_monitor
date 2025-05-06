import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_monitor',
            executable='system_monitor',
            name='system_monitor',
            output='screen',
            parameters=[{'topic_path': 'pc1'}],
        ),
        # 多个节点就新增Node即可...
    ])
