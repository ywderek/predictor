from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('armor_tracker')
    config = os.path.join(share, 'config', 'default.yaml')       # 卡尔曼参数
    rviz_config = os.path.join(share, 'config', 'tune.rviz')     # rviz布局（含视频面板）
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # 播 bag 时必须 true（bag 要加 --clock）；接真机时传 use_sim_time:=false
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),   # rviz:=false 可关

        # 订阅节点（滤波器主体），卡尔曼参数从 config/default.yaml 加载
        Node(
            package='armor_tracker',
            executable='subscriber',
            name='vision_subscriber',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),

        # 压缩图流解压成原始图流，供 rviz Image 面板显示
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher',
            parameters=[{'in_transport': 'compressed',
                         'out_transport': 'raw',
                         'use_sim_time': use_sim_time}],
            remappings=[('in/compressed', '/camera/image_raw/compressed'),
                        ('out', '/camera/image_raw')],
        ),

        # rviz，加载预设布局（3D视图 + 视频面板 + MarkerArray）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
