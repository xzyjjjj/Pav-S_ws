'''
相机 + yolo + 距离检测 + 语义costmap 相关节点激活启动 Launch 文件

'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
# ------------------------------------

def generate_launch_description():

    # ---------------- 1. 声明参数 ----------------

    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera', default_value='true', description='Whether to launch the Astra camera'
    )


    # ---------------- 2. 定义第一批启动的节点 ----------------
    # (底盘, TF, LIO, 和其他应用节点)

    yolo_detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yolo_detector'), 'launch', 'yolo_detect.launch.py')
        ])
    )

    distance_detector_node = Node(
        package='distance_detector',
        executable='distance_node_cv',
        output='screen'
    )

    semantic_node = Node(
        package='costmap_process',
        executable='senamic_node',
        output='screen'
    )
    costmap_pub_node = Node(
        package='costmap_process',
        executable='costmap_pub_node',
        output='screen'
    )

    astra_camera_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            os.path.join(get_package_share_directory('astra_camera'), 'launch', 'astro_pro_plus.launch.xml')
        ]),
        launch_arguments={
            'enable_point_cloud': 'false', 'enable_depth': 'false', 'enable_ir': 'false', 
            'enable_color': 'true', 'depth_registration': 'false'
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_camera'))
    )

    # ---------------- 7. 组合并返回 LaunchDescription ----------------

    return LaunchDescription([
        # 声明所有参数
        launch_camera_arg,

        # 立即启动第一批节点
        yolo_detect_launch,
        astra_camera_launch,
        distance_detector_node,
        semantic_node,
        costmap_pub_node
    ])