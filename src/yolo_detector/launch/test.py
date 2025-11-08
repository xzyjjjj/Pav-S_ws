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

print(get_package_share_directory('livox_ros_driver2'))