# launch/mission_manager.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_detector') # 假设在同一个包
    
    params_file = os.path.join(pkg_dir, 'config', 'mission_manager_params.yaml')

    mission_manager_node = Node(
        package='yolo_detector', # 假设在同一个包
        executable='mission_manager',    # 新的 Python 脚本
        name='mission_manager',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        mission_manager_node
    ])