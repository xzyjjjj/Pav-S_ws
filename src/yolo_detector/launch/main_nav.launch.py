'''
底盘 + TF + LIO + Nav2 + map_server + Livox MID360 激活启动 Launch 文件

'''



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent,
    TimerAction, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

# --- 用于生命周期管理的关键 imports ---
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events.matchers import matches_action
from lifecycle_msgs.msg import Transition
# ------------------------------------

def generate_launch_description():

    # ---------------- 1. 声明参数 ----------------

    map_yaml_file_arg = DeclareLaunchArgument(
        'map_yaml_file',
        default_value='/Map_yaml/circuit/map_huandao.yaml',
        description='Full path to map file to load'
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'
        ),
        description='Full path to the NAV2 params file to use'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to start RVIZ'
    )
    launch_lidar_arg = DeclareLaunchArgument(
        'launch_lidar', default_value='true', description='Whether to launch the Livox MID360'
    )

    # --- 新增：传感器启动延迟参数 ---
    sensor_delay_arg = DeclareLaunchArgument(
        'sensor_startup_delay',
        default_value='10.0', # 默认延迟10秒
        description='Delay in seconds after Nav2 launch before starting sensors (Lidar/Camera)'
    )
    # ------------------------------------


    # ---------------- 2. 定义第一批启动的节点 ----------------
    # (底盘, TF, LIO, 和其他应用节点)
    
    base_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turn_on_wheeltec_robot'), 'launch', 'base_serial.launch.py')
        ])
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['4.6', '0.81', '0.20', '3.1415926', '0', '0', 'map', 'camera_init']
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
        ])
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/nav2_default_view.rviz'],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # ---------------- 3. 定义 Map Server (第二阶段) ----------------
    
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_yaml_file')}]
    )

    on_map_server_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=map_server_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ]
        )
    )

    on_map_server_configure_success = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=map_server_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ]
        )
    )

    # ---------------- 4. 定义 Nav2 (第三阶段) ----------------
    
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch_ROS2', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_map_server': 'false', 
            'map_subscribe_transient_local': 'true',
            'autostart': 'true',
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    # ---------------- 5. 定义传感器 (第四阶段 - Lidar/Camera) ----------------

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch', 'msg_MID360_map_launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('launch_lidar'))
    )


    # [事件 D]：定义一个计时器，用于启动传感器
    # --- 修改：使用 LaunchConfiguration 读取延迟时间 ---
    delayed_sensors_launch = TimerAction(
        period=LaunchConfiguration('sensor_startup_delay'),
        actions=[
            LogInfo(msg='Nav2 startup delay complete. Launching Lidar and Camera.'),
            livox_launch
        ]
    )
    # ---------------------------------------------

    # ---------------- 6. 定义启动链 (核心) ----------------

    # [事件 C]：当 map_server 'activate' 成功 (进入 'active' 状态) 时...
    on_map_server_activate_success = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=map_server_node,
            start_state='activating',
            goal_state='active',
            entities=[
                LogInfo(msg='Map server is active. Launching Nav2 and starting sensor delay timer.'),
                # 1. 启动 Nav2
                nav2_bringup_launch,
                # 2. 启动传感器的倒计时
                delayed_sensors_launch
            ]
        )
    )

    # ---------------- 7. 组合并返回 LaunchDescription ----------------

    return LaunchDescription([
        # 声明所有参数
        map_yaml_file_arg,
        nav2_params_file_arg,
        use_rviz_arg,
        launch_lidar_arg,
        sensor_delay_arg, # <-- 添加新声明的参数

        # 立即启动第一批节点
        base_serial_launch,
        static_tf_node,
        fast_lio_launch,
        rviz_node,

        # 立即启动 map_server
        map_server_node,
        
        # 注册所有事件处理器
        on_map_server_start,
        on_map_server_configure_success,
        on_map_server_activate_success
    ])