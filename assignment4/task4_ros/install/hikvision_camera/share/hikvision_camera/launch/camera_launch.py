from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包目录
    pkg_dir = get_package_share_directory('hikvision_camera')
    
    return LaunchDescription([
        # 声明相机的参数
        DeclareLaunchArgument(
            'serial_number', 
            default_value='',
            description='Camera Serial Number'
        ),
        DeclareLaunchArgument(
            'image_format', 
            default_value='BGR8', 
            description='Image format'
        ),
        DeclareLaunchArgument(
            'exposure_time', 
            default_value='10000', 
            description='Exposure Time in Microseconds'
        ),
        DeclareLaunchArgument(
            'gain', 
            default_value='10', 
            description='Gain'
        ),
        DeclareLaunchArgument(
            'frame_rate', 
            default_value='30', 
            description='Frame rate (fps)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),

        # 启动相机节点
        Node(
            package='hikvision_camera',
            executable='hikvision_camera_node',
            name='hikvision_camera',
            output='screen',
            parameters=[{
                'serial_number': LaunchConfiguration('serial_number'),
                'image_format': LaunchConfiguration('image_format'),
                'exposure_time': LaunchConfiguration('exposure_time'),
                'gain': LaunchConfiguration('gain'),
                'frame_rate': LaunchConfiguration('frame_rate'),
            }]
        ),

        # 静态 TF 发布器（确保 RViz 能显示图像）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            arguments=['0', '0', '1.5', '0', '0', '0', 'map', 'camera_frame']
        ),

        # 启动RViz节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'vision_task.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen'
        ),

        # 启动日志
        LogInfo(
            msg=['Hikvision Camera Node started with serial: ', LaunchConfiguration('serial_number')]
        )
    ])