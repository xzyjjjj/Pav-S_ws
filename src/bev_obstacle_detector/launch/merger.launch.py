# launch/merger.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 获取参数文件路径
    pkg_dir = get_package_share_directory('bev_obstacle_detector')
    params_file = os.path.join(pkg_dir, 'config', 'merger_params.yaml')

    pointcloud_merger_node = Node(
        package='bev_obstacle_detector', # 请确保这个包名是正确的
        executable='pointcloud_merger_node',
        name='pointcloud_merger_node',
        output='screen',
        # 加载 YAML 参数
        parameters=[params_file]
    )

    return LaunchDescription([
        pointcloud_merger_node
    ])