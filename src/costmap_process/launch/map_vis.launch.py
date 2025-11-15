#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Map Visualization Launch File

启动地图和代价地图可视化发布节点，用于在RViz中显示不同的地图和代价地图，
而不影响导航使用的地图（原来的map_server和global_costmap）。
"""

from launch import LaunchDescription
from launch import conditions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明启动参数
    yaml_filename_arg = DeclareLaunchArgument(
        'yaml_filename',
        default_value='',
        description='要加载的地图YAML文件路径（必须设置）'
    )
    
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/map_vis',
        description='发布地图的话题名称（默认: /map_vis）'
    )
    
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/costmap_vis',
        description='发布代价地图的话题名称（默认: /global_costmap/costmap_vis）'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='地图的frame_id（默认: map）'
    )
    
    publish_map_arg = DeclareLaunchArgument(
        'publish_map',
        default_value='true',
        description='是否发布静态地图（默认: true）'
    )
    
    publish_costmap_arg = DeclareLaunchArgument(
        'publish_costmap',
        default_value='true',
        description='是否发布代价地图（默认: true）'
    )
    
    # 创建静态地图发布节点
    map_vis_node = Node(
        package='costmap_process',
        executable='map_vis_publisher',
        name='map_vis_publisher',
        output='screen',
        condition=conditions.IfCondition(LaunchConfiguration('publish_map')),
        parameters=[{
            'yaml_filename': LaunchConfiguration('yaml_filename'),
            'topic_name': LaunchConfiguration('map_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )
    
    # 创建代价地图发布节点
    costmap_vis_node = Node(
        package='costmap_process',
        executable='costmap_vis_publisher',
        name='costmap_vis_publisher',
        output='screen',
        condition=conditions.IfCondition(LaunchConfiguration('publish_costmap')),
        parameters=[{
            'yaml_filename': LaunchConfiguration('yaml_filename'),
            'topic_name': LaunchConfiguration('costmap_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )
    
    return LaunchDescription([
        yaml_filename_arg,
        map_topic_arg,
        costmap_topic_arg,
        frame_id_arg,
        publish_map_arg,
        publish_costmap_arg,
        map_vis_node,
        costmap_vis_node,
    ])

