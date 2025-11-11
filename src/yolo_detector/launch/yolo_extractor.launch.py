# launch/yolo_extractor.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_detector') 
    
    # 指向新的参数文件
    params_file = os.path.join(pkg_dir, 'config', 'yolo_extractor_params.yaml')

    yolo_extractor_node = Node(
        package='yolo_detector', 
        executable='yolo_goal_extractor', 
        name='yolo_goal_extractor',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        yolo_extractor_node
    ])