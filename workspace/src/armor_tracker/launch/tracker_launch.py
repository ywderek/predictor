from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('armor_tracker'),
        'config',
        'default.yaml')

    rviz_config = os.path.join(
        get_package_share_directory('armor_tracker'),
        'config',
        'view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),   # rviz:=false 可关

        # 订阅节点（滤波器主体），卡尔曼参数从 config/default.yaml 加载
        Node(
            package='armor_tracker',
            executable='subscriber',
            name='vision_subscriber',
            output='screen',
            parameters=[config],
        ),

        # rviz，加载预设配置（视频 + 标记球 + tf）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])

