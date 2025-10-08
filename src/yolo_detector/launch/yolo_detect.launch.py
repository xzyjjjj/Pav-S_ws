import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 找到 yolo_detector 包的共享目录，用于加载基础参数文件
    yolo_detector_share_dir = get_package_share_directory('yolo_detector')
    # 找到 yolov5_lib 包的共享目录，用于定位模型和配置文件
    yolov5_lib_share_dir = get_package_share_directory('yolov5_lib')
    # --- 动态构建跨包文件路径 ---
    weights_path = PathJoinSubstitution(
        [yolov5_lib_share_dir, 'runs/train','exp4', 'weights/best.pt']
    )
    config_path = PathJoinSubstitution(
        [yolov5_lib_share_dir, 'config', 'competition_dataset.yaml']
    )


    # 1. 定位到参数文件
    params_file = os.path.join(
        get_package_share_directory('yolo_detector'), #<-- 包名
        'config',
        'params.yaml' #<-- 参数文件名
    )

    # 2. 定义要启动的节点
    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_node',      #<-- 你在setup.py里定义的'可执行文件名'
        name='yolo_node',           #<-- 你希望节点在ROS网络中显示的名字
        parameters=[            
            params_file,          # 1. 首先加载基础参数文件
            {                     # 2. 然后，用动态生成的正确路径来覆盖占位符
                'weights_path': weights_path,
                'config_path': config_path,
            }
        ],   #<-- 把参数文件传给节点
        output='screen'             #<-- 将节点的日志输出到终端，方便调试
    )

    # 3. 返回LaunchDescription，其中包含了所有要启动的节点
    return LaunchDescription([
        yolo_node
    ])



def generate_launch_description():

    # 找到 yolo_detector 包的共享目录，用于加载基础参数文件
    yolo_detector_share_dir = get_package_share_directory('yolo_detector')
    
    # 找到 yolov5_lib 包的共享目录，用于定位模型和配置文件
    yolov5_lib_share_dir = get_package_share_directory('yolov5_lib')

    # --- 动态构建跨包文件路径 ---
    weights_path = PathJoinSubstitution(
        [yolov5_lib_share_dir, 'weights', 'best.pt']
    )
    config_path = PathJoinSubstitution(
        [yolov5_lib_share_dir, 'config', 'competition_dataset.yaml']
    )

    # 基础参数文件的路径
    params_file = os.path.join(yolo_detector_share_dir, 'config', 'params.yaml')

    # 定义节点
    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[
            params_file,          # 1. 首先加载基础参数文件
            {                     # 2. 然后，用动态生成的正确路径来覆盖占位符
                'weights_path': weights_path,
                'config_path': config_path,
            }
        ]
    )

    return LaunchDescription([
        yolo_node
    ])