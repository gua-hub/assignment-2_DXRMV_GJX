from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包目录
    pkg_dir = get_package_share_directory('hikvision_camera')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'image_path',
            default_value=os.path.expanduser('~/test_images/test_image_with_text.jpg'),
            description='Path to test image for virtual camera'
        ),
        DeclareLaunchArgument(
            'frame_rate',
            default_value='30',
            description='Frame rate for virtual camera'
        ),

        # 虚拟相机节点（使用image_publisher）
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='virtual_camera',
            parameters=[{
                'filename': LaunchConfiguration('image_path'),
                'publish_rate': LaunchConfiguration('frame_rate'),
                'frame_id': 'camera_frame',  # 这里设置框架ID
                'use_sim_time': False
            }],
            remappings=[
                ('image', '/camera/image_raw')  # 重映射话题
            ],
            output='screen'
        ),

        # 静态 TF 发布器 - 新增部分
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_frame']  # 从map到camera_frame
        ),

        # 您的相机节点（处理帧率发布和参数管理）
        Node(
            package='hikvision_camera',
            executable='hikvision_camera_node',
            name='hikvision_camera',
            output='screen',
            parameters=[{
                'use_virtual_camera': True,  # 告诉节点使用虚拟模式
                'frame_rate': LaunchConfiguration('frame_rate'),
                'exposure_time': 10000,
                'gain': 10,
                'image_format': 'BGR8',
                'serial_number': 'VIRTUAL_CAMERA_001'  # 虚拟序列号
            }]
        ),

        # RViz节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'vision_task.rviz')],
            output='screen'
        ),

        # 启动日志
        LogInfo(
            msg=['Virtual camera test started with image: ', LaunchConfiguration('image_path')]
        ),
        LogInfo(
            msg=['Target frame rate: ', LaunchConfiguration('frame_rate'), ' fps']
        )
    ])