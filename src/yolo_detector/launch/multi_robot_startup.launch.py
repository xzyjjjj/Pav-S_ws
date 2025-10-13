import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    """
    生成一个LaunchDescription对象，用于启动机器人和导航相关节点。

    该启动文件会：
    1. 为每个主要命令打开一个独立的xterm终端。
    2. 自动管理map_server的生命周期（configure -> activate）。
    3. 在其他所有节点启动后，最后启动Livox雷达驱动。
    """
    # --- 配置 ---
    # 定义用于在新终端中启动命令的前缀。可根据你的系统更换 (e.g., 'gnome-terminal -- ')
    terminal_prefix = 'xterm -e'
    
    # 地图文件的绝对路径，请确保路径正确
    map_yaml_path = '/Map_yaml/crossing/map_chalu.yaml'

    # --- 定义所有需要执行的动作 ---

    # 1. 启动机器人底层驱动
    # 使用ExecuteProcess可以确保'prefix'参数生效
    base_serial_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'turn_on_wheeltec_robot', 'base_serial.launch.py'],
        prefix=terminal_prefix,
        shell=True,
        name='wheeltec_base'
    )

    # 2. 启动地图服务器节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        prefix=terminal_prefix,
        parameters=[{'yaml_filename': map_yaml_path}]
    )

    # 3. 配置map_server的命令
    configure_map_server_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        shell=True,
        name='configure_map_server'
    )

    # 4. 激活map_server的命令
    activate_map_server_cmd = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
        shell=True,
        name='activate_map_server'
    )

    # 5.1 启动静态坐标变换发布器
    static_tf_pub_node1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_camera',
        output='screen',
        prefix=terminal_prefix,
        arguments=['4.6', '0.77', '0.25', '3.1415926', '0', '0', 'map', 'camera_init']
    )

    # 5.2 启动静态坐标变换发布器
    static_tf_pub_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_body_to_cameralink',
        output='screen',
        prefix=terminal_prefix,
        arguments=['0.11365', '0.00016542', '0.076204', '0', '0', '0', 'body', 'camera_link']
    )

    # 6. 启动Fast-LIO
    fast_lio_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'fast_lio', 'mapping.launch.py'],
        prefix=terminal_prefix,
        shell=True,
        name='fast_lio_mapping'
    )

    # 7. 启动Navigation2
    navigation_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py'],
        prefix=terminal_prefix,
        shell=True,
        name='nav2_bringup'
    )

    # 8. 启动Livox雷达驱动 (此命令会被事件处理器延迟启动)
    livox_driver_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_map_launch.py'],
        prefix=terminal_prefix,
        shell=True,
        name='livox_driver'
    )

    # --- 定义事件处理器以管理启动顺序 ---

    # 事件1: 当map_server_node进程启动后，执行configure命令
    on_map_server_start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=map_server_node,
            on_start=[
                LogInfo(msg='Map server started. Configuring...'),
                configure_map_server_cmd
            ]
        )
    )

    # 事件2: 当configure命令成功退出后，执行activate命令
    on_configure_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=configure_map_server_cmd,
            # condition=lambda event: event.return_code == 0, # 可选：仅在成功时执行
            on_exit=[
                LogInfo(msg='Map server configured. Activating...'),
                activate_map_server_cmd
            ]
        )
    )

    # 事件3: 当Navigation2进程启动后，再启动Livox雷达驱动
    # 我们选择navigation_cmd作为“其他所有命令”的代表
    on_navigation_start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=navigation_cmd,
            on_start=[
                LogInfo(msg='Core systems started. Launching Livox driver...'),
                livox_driver_cmd
            ]
        )
    )

    # --- 返回LaunchDescription ---
    # 将所有事件处理器和需要立即启动的动作放入列表中
    return LaunchDescription([
        # 事件处理器
        on_map_server_start_handler,
        on_configure_exit_handler,
        on_navigation_start_handler,

        # 立即启动的动作 (其他动作由事件触发)
        base_serial_cmd,
        map_server_node,
        static_tf_pub_node1,
        static_tf_pub_node2,
        fast_lio_cmd,
        navigation_cmd,
    ])