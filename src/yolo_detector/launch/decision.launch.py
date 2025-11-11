import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('yolo_detector')

    # 1. 包含 yolo_extractor.launch.py
    yolo_extractor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'yolo_extractor.launch.py')
        )
    )

    # 2. 包含 mission_manager.launch.py
    mission_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'mission_manager.launch.py')
        )
    )

    # 3. 返回包含这两个子 launch 文件的 LaunchDescription
    return LaunchDescription([
        yolo_extractor_launch,
        mission_manager_launch
    ])