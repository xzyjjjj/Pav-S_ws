import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# 导入 DeclareLaunchArgument 和 LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 定义一个 Launch Argument，允许用户从命令行设置 'enable_vis'
    vis_arg = DeclareLaunchArgument(
        'enable_vis',
        default_value='False', # 默认值为 False (字符串形式)
        description='Set to True to enable OpenCV visualization windows.'
    )
    
    # 2. 获取 Launch Argument 的配置值
    enable_vis = LaunchConfiguration('enable_vis')
    
    pkg_dir = get_package_share_directory('bev_obstacle_detector')
    params_file = os.path.join(pkg_dir, 'config', 'bev_params.yaml')

    ipm_node = Node(
        package='bev_obstacle_detector',
        executable='ipm_node',
        name='ipm_node',
        output='screen',
        # 3. 将 Launch Argument 的值作为参数传递给节点
        #    注意：这里使用字典，它将覆盖 YAML 文件中 'enable_vis' 的任何设置
        parameters=[
            params_file,
            {'enable_vis': enable_vis}
        ]
    )

    # # 启动点云融合
    # pointcloud_merger_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(pkg_dir, 'launch', 'merger.launch.py')
    #     ])
    # )

    return LaunchDescription([
        vis_arg, # 必须将 Argument 添加到 LaunchDescription 中
        ipm_node,
        # pointcloud_merger_launch
    ])