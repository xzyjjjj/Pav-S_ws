#!/usr/bin/env python3
# coding=utf-8

import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32, Bool
from enum import Enum
from typing import Optional
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from vision_msgs.msg import Pose2D, Point2D
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import time
# from red_segment_msg.msg import ObstacleInfo

class RobotState(Enum):
    CLEAR = "CLEAR"           # 无危险，全速
    STOPPING = "STOPPING"     # 停车线触发，减速中
    HOLDING = "HOLDING"       # 保持停止
    RED_STOPPING = "RED_STOPPING" # 红色停车触发，减速中
    RED_HOLDING = "RED_HOLDING"   # 红色停车保持停止

class CmdVelLimiter(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_limiter')

        # --- 默认配置 ---
        # 停车后冷却时间（秒）: 在停完车后这段时间内不再触发停车
        self.declare_parameter('stop_line_cooldown', 5.0)
        self.stop_line_cooldown = float(self.get_parameter('stop_line_cooldown').get_parameter_value().double_value)

        # Topics
        self.declare_parameter('input_cmd_vel', '/cmd_vel_nav2')
        self.declare_parameter('output_cmd_vel', '/cmd_vel')
        self.declare_parameter('stop_line_topic', '/object_in_proximity')   # Bool topic
        self.declare_parameter('obstacle_topic', '/bev/obstacle_info')     # optional Float32 topic

        self.declare_parameter('red_stop_topic', '/object_in_bev')   # Bool topic for red stop detection
        self.declare_parameter('scane_topic', '/scan')   # LaserScan topic for obstacle detection

        # Speed caps (m/s, rad/s)
        self.declare_parameter('max_linear_clear', 0.35)
        self.declare_parameter('min_linear', 0.0)

        # 停车线检测参数
        self.declare_parameter('stop_line_hold_time', 2.0)       # HOLDING 状态停留时间（秒）
        self.declare_parameter('stop_line_trigger_time', 0.5)   # 识别后等待触发停车时间
        self.declare_parameter('image_height_px', 480)

        # 红色停车检测参数
        self.declare_parameter('red_stop_hold_time', 1.0)
        self.declare_parameter('red_stop_trigger_time', 0.1)
        # self.declare_parameter('red_stop_cooldown', 5.0)
        

        # --- 读取参数 ---
        in_topic = self.get_parameter('input_cmd_vel').get_parameter_value().string_value
        out_topic = self.get_parameter('output_cmd_vel').get_parameter_value().string_value
        stop_topic = self.get_parameter('stop_line_topic').get_parameter_value().string_value
        # obstacle_topic = self.get_parameter('obstacle_topic').get_parameter_value().string_value
        red_stop_topic = self.get_parameter('red_stop_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scane_topic').get_parameter_value().string_value


        self.max_linear_clear = float(self.get_parameter('max_linear_clear').get_parameter_value().double_value)
        self.min_linear = float(self.get_parameter('min_linear').get_parameter_value().double_value)
        self.stop_line_hold_time = float(self.get_parameter('stop_line_hold_time').get_parameter_value().double_value)
        self.stop_line_trigger_time = float(self.get_parameter('stop_line_trigger_time').get_parameter_value().double_value)
        self.image_height_px = int(self.get_parameter('image_height_px').get_parameter_value().integer_value)
        
        self.red_stop_hold_time = float(self.get_parameter('red_stop_hold_time').get_parameter_value().double_value)
        self.red_stop_trigger_time = float(self.get_parameter('red_stop_trigger_time').get_parameter_value().double_value)
        # self.red_stop_cooldown = self.stop_line_cooldown  

        # 状态机变量
        self.current_state = RobotState.CLEAR
        self.state_start_time: Optional[Time] = None
        self.last_stop_time: Optional[Time] = None   # 最近一次进入 HOLDING（停车完成）时的时间，作为 cooldown 基准
        self.latest_stopline_seen_time: Optional[Time] = None  # 最近一次收到 stop_line True 的时间
        self.stop_line_active = False   # 当前是否检测到 stop line (瞬时)
        self.cmd_x = None
        self.scan_msg = None

        self.red_stop_active = False
        self.latest_redstop_seen_time: Optional[Time] = None
        self.last_redstop_time: Optional[Time] = None
        

        # QoS
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        det_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        # IO
        self.cmd_sub = self.create_subscription(Twist, in_topic, self.on_cmd_vel, cmd_qos)
        self.cmd_pub = self.create_publisher(Twist, out_topic, cmd_qos)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, det_qos)

        # stop line: expect std_msgs/Bool topic (True when detected)
        if stop_topic:
            self.stop_sub = self.create_subscription(Bool, stop_topic, self.on_stopline, det_qos)
        else:
            self.stop_sub = None

        if red_stop_topic:
            self.red_stop_sub = self.create_subscription(Bool, red_stop_topic, self.on_red_stop, det_qos)
        else:
            self.red_stop_sub = None

        # # optional obstacle distance topic
        # if obstacle_topic:
        #     self.obstacle_sub = self.create_subscription(ObstacleInfo, obstacle_topic, self.on_obstacle, det_qos)
        # else:
        #     self.obstacle_sub = None

        self.get_logger().info(f'CmdVelLimiter started. in={in_topic} out={out_topic} stop_topic={stop_topic or "disabled"}')

    # -----------------------------------------
    # 回调：停车线 topic（std_msgs/Bool）
    def on_stopline(self, msg: Bool) -> None:
        now = self.get_clock().now()
        if msg.data and self.cmd_x is not None:
            # 首次看到 True，并且当前并非是初始状态，记录时间（用于 trigger_time ）
            if self.latest_stopline_seen_time is None:
                self.latest_stopline_seen_time = now
            # 标记为当前检测到停车线（瞬时）
            self.stop_line_active = True
        else:
            # 看到 False，清除检测（reset）
            self.latest_stopline_seen_time = None
            self.stop_line_active = False

        # 每次 stop topic 更新都调用状态机（带当前检测情况）
        self.update_state_machine(saw_obstacle=False, max_obstacle_height_px=0.0, now=now)

    # def on_obstacle(self, msg: ObstacleInfo) -> None:
    #     self.obstacle_info = msg

    def on_scan(self, msg: LaserScan) -> None:
        self.scan_msg = msg


    def check_scan_empty(self) -> bool:
        """
        检查雷达数据是否为空（即是否所有点都在 range 之外）。
        如果 scan_msg 为 None，或者 ranges 里全是 inf，返回 True。
        """
        # # 计时开始
        # start_time = time.perf_counter()
        if self.scan_msg is None:
            return True

        if not self.scan_msg.ranges:
            return True
        min_dist = min(self.scan_msg.ranges)
        is_all_inf = (min_dist == float('inf'))
        
        # # return is_all_inf
        # ranges = np.array(self.scan_msg.ranges)
        # finite_ranges = ranges[np.isfinite(ranges)]
        
        # # 计时结束
        # end_time = time.perf_counter()
        # elapsed_time_ms = (end_time - start_time) * 1000

        # if len(finite_ranges) == 0:
        #     is_empty = True
        # else:
        #     is_empty = False     
    
        # # --- 输出日志 ---
        # if is_all_inf:
        #     self.get_logger().info(
        #         f"✅ Scan Check: EMPTY (All points are inf/NaN). Took: {elapsed_time_ms:.4f} ms"
        #     )
        # else:
        #     self.get_logger().info(
        #         f"❌ Scan Check: NOT EMPTY points are valid). Took: {elapsed_time_ms:.4f} ms"
        #     )
        return is_all_inf


    def on_red_stop(self, msg: Bool) -> None:
        now = self.get_clock().now()
        if msg.data:
            if self.latest_redstop_seen_time is None:
                self.latest_redstop_seen_time = now
            self.red_stop_active = True
        else:
            self.latest_redstop_seen_time = None
            self.red_stop_active = False

        self.update_state_machine(saw_obstacle=False, max_obstacle_height_px=0.0, now=now)


    # 状态机逻辑核心
    def update_state_machine(self, saw_obstacle: bool, max_obstacle_height_px: float, now: Time) -> None:
        # 1) 检查停车线触发条件（持续检测 + cooldown 过滤）
        stop_line_triggered = False
        # 如果当前检测到停车线且持续时间超过 trigger_time，则视为触发
        if self.stop_line_active and self.latest_stopline_seen_time is not None:
            dt = (now - self.latest_stopline_seen_time).nanoseconds / 1e9
            if dt >= self.stop_line_trigger_time:
                # 还需检查 cooldown：上次停车结束后，距现在是否 >= cooldown
                if self.last_stop_time is None:
                    stop_line_triggered = True
                else:
                    since_last_stop = (now - self.last_stop_time).nanoseconds / 1e9
                    if since_last_stop >= self.stop_line_cooldown:
                        stop_line_triggered = True
        
        # 2) 检测红色停车触发条件（持续检测+cooldown过滤）
        red_stop_triggered = False
        if self.red_stop_active and self.latest_redstop_seen_time is not None:
            dt = (now - self.latest_redstop_seen_time).nanoseconds / 1e9
            # 障碍物进入鸟瞰图区域 + scan还未扫到
            if dt >= self.red_stop_trigger_time and self.check_scan_empty():
                red_stop_triggered = True
                


        if self.current_state == RobotState.CLEAR:
            if stop_line_triggered:
                self.get_logger().info('Stop line triggered -> STOPPING')
                self.current_state = RobotState.STOPPING
                self.state_start_time = now
            elif red_stop_triggered:
                self.get_logger().info('Red stop triggered -> RED_STOPPING')
                self.current_state = RobotState.RED_STOPPING
                self.state_start_time = now

        elif self.current_state == RobotState.STOPPING:
            # STOPPING 表示开始减速/停车，立即转为 HOLDING（模拟减速完成）
            # 在真实系统你可能想等一段减速时间再进入 HOLDING
            self.get_logger().info('Entering HOLDING (stopped)')
            self.current_state = RobotState.HOLDING
            self.state_start_time = now
            # 记录停车完成时间，用作后续 cooldown 判断
            self.last_stop_time = now
            # clear instantaneous stop-line detection so it won't retrigger immediately
            self.latest_stopline_seen_time = None
            self.stop_line_active = False

        elif self.current_state == RobotState.HOLDING:
            # 在 HOLDING 状态保持 stop_line_hold_time 秒
            dt = (now - self.state_start_time).nanoseconds / 1e9 if self.state_start_time else 0.0
            if dt >= self.stop_line_hold_time:
                self.get_logger().info('HOLDING -> CLEAR')
                self.current_state = RobotState.CLEAR
                self.state_start_time = now

        elif self.current_state == RobotState.RED_STOPPING:
            self.get_logger().info('Entering RED_HOLDING (stopped for red)')
            self.current_state = RobotState.RED_HOLDING
            self.state_start_time = now
            self.last_redstop_time = now
            self.latest_redstop_seen_time = None
            self.red_stop_active = False

        elif self.current_state == RobotState.RED_HOLDING:
            dt = (now - self.state_start_time).nanoseconds / 1e9 if self.state_start_time else 0.0
            if dt >= self.red_stop_hold_time:
                self.get_logger().info('RED_HOLDING -> CLEAR')
                self.current_state = RobotState.CLEAR
                self.state_start_time = now

    # cmd_vel 回调：根据状态应用限速并发布
    def on_cmd_vel(self, msg: Twist) -> None:
        now = self.get_clock().now()
        if self.current_state == RobotState.CLEAR:
            desired_cap = self.max_linear_clear
        elif self.current_state == RobotState.STOPPING or self.current_state == RobotState.RED_STOPPING:
            desired_cap = self.min_linear
        elif self.current_state == RobotState.HOLDING or self.current_state == RobotState.RED_HOLDING:
            desired_cap = self.min_linear
        else:
            desired_cap = self.max_linear_clear

        scaled = Twist()
        lin_x = msg.linear.x
        lin_y = msg.linear.y
        lin_z = msg.linear.z
        lin_magnitude = math.hypot(lin_x, lin_y)
        self.cmd_x = msg.linear.x

        # 只在调试或必要时打印，防止日志被大量占用
        # self.get_logger().debug(f"lin_speed:{lin_magnitude:.3f} cap={desired_cap:.3f}")

        if lin_magnitude > desired_cap and lin_magnitude > 1e-6:
            lin_scale = desired_cap / lin_magnitude
        else:
            lin_scale = 1.0

        scaled.linear.x = lin_x * lin_scale
        scaled.linear.y = lin_y * lin_scale
        scaled.linear.z = lin_z

        ang_x = msg.angular.x
        ang_y = msg.angular.y
        ang_z = msg.angular.z

        if self.current_state == RobotState.HOLDING:
            scaled.angular.z = 0.0
        else:   
            scaled.angular.z = ang_z
            # # 检查是否接近障碍物，如果接近调整角速度，并且降低线速度
            # if  hasattr(self, 'obstacle_info') and self.obstacle_info.find:
            #     scaled.angular.z += self.obstacle_info.angular_change
            #     # scaled.linear.x = min(scaled.linear.x * 0.4, 0.2)
            #     # scaled.linear.y = min(scaled.linear.y * 0.4, 0.2)
            #     self.get_logger().info(f"Obstacle detected, adjusting speeds.")
                

        scaled.angular.x = ang_x
        scaled.angular.y = ang_y

        self.cmd_pub.publish(scaled)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelLimiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
