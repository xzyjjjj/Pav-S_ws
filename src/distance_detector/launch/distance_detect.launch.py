import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.substitutions import PathJoinSubstitution  <-- 这个脚本中并未使用，可以移除
from launch.actions import DeclareLaunchArgument       # <--- 1. 导入 DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration   # <--- 2. 导入 LaunchConfiguration

def generate_launch_description():

    distance_detector_share_dir = get_package_share_directory('distance_detector')
    params_file = os.path.join(
        distance_detector_share_dir,
        'config',
        'params.yaml'
    )

    enable_vis_arg = DeclareLaunchArgument(
        'enable_vis',  #<-- 参数的名称
        default_value='true', #<-- 默认值
        description='Enable visualization' #<-- 描述信息
    )

    detector_node = Node(
        package='distance_detector',
        executable='distance_node_cv',  # 替换为您的可执行文件名
        name='distance_node_cv',     # 这是您在 __init__ 中设置的节点名
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file, # <-- 首先加载文件中的所有参数
            {'enable_vis': LaunchConfiguration('enable_vis')} 
        ]
    )
    
    return LaunchDescription([
        enable_vis_arg, 
        detector_node
    ])