import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.substitutions import PathJoinSubstitution  <-- 这个脚本中并未使用，可以移除
from launch.actions import DeclareLaunchArgument       # <--- 1. 导入 DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration   # <--- 2. 导入 LaunchConfiguration

def generate_launch_description():
    
    # 找到 yolo_detector 包的共享目录，用于加载基础参数文件
    yolo_detector_share_dir = get_package_share_directory('yolo_detector')
    
    # 1. 定位到参数文件
    params_file = os.path.join(
        yolo_detector_share_dir,
        'config',
        'yolo_params.yaml'
    )

    # --- 关键改动点 开始 ---

    # 3. 声明 enable_vis 启动参数
    #    我们在这里定义一个可以在命令行中设置的参数
    enable_vis_arg = DeclareLaunchArgument(
        'enable_vis',  #<-- 参数的名称
        default_value='true', #<-- 默认值
        description='Enable visualization' #<-- 描述信息
    )

    # 4. 定义要启动的节点
    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_node',
        name='yolo_node',
        parameters=[
            params_file, # <-- 首先加载文件中的所有参数
            
            # <-- 然后，我们在这里覆盖或添加 enable_vis 参数
            # 节点内的参数名将是 'enable_vis'
            # 它的值将来自我们上面声明的 'enable_vis' 启动参数
            {'enable_vis': LaunchConfiguration('enable_vis')} 
        ],
        output='screen'
    )

    # 5. 返回LaunchDescription
    return LaunchDescription([
        enable_vis_arg,  # <--- 必须将声明的参数添加到返回列表中
        yolo_node
    ])
    
    # --- 关键改动点 结束 ---