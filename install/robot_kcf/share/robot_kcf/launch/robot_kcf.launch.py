from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    camera_bringup_dir = get_package_share_directory('astra_camera')
    launch_dir = os.path.join(bringup_dir, 'launch')

    astra_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(camera_bringup_dir, 'launch', 'astro_pro_plus.launch.xml')),
    )
    robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    return LaunchDescription([
    astra_camera,
    robot,
    Node(
        package='robot_kcf',
        executable='run_tracker_node',
        parameters=[{'targetDist_': 0.6}],
        output='screen',
    )
    
    ])

