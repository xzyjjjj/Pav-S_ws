#!/usr/bin/env python3
# coding=utf-8

import math
import rclpy
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
from red_segment_msg.msg import ObstacleInfo

class RobotState(Enum):
    CLEAR = "CLEAR"           # 无危险，全速
    STOPPING = "STOPPING"     # 停车线触发，减速中
    HOLDING = "HOLDING"       # 保持停止

class CmdVelLimiter(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_limiter')

        # --- 默认配置 ---
        # 停车后冷却时间（秒）: 在停完车后这段时间内不再触发停车
        self.declare_parameter('stop_line_cooldown', 3.0)
        self.stop_line_cooldown = float(self.get_parameter('stop_line_cooldown').get_parameter_value().double_value)

        # Topics
        self.declare_parameter('input_cmd_vel', '/cmd_vel_nav2')
        self.declare_parameter('output_cmd_vel', '/cmd_vel')
        self.declare_parameter('stop_line_topic', '/object_in_proximity')   # Bool topic
        self.declare_parameter('obstacle_topic', '/obstacle_cv')     # optional Float32 topic
        # self.declare_parameter('detections_topic', '')            # optional Detection2DArray topic (if used)

        # Speed caps (m/s, rad/s)
        self.declare_parameter('max_linear_clear', 0.35)
        self.declare_parameter('min_linear', 0.0)

        # 停车线检测参数
        self.declare_parameter('stop_line_hold_time', 2.0)       # HOLDING 状态停留时间（秒）
        self.declare_parameter('stop_line_trigger_time', 0.5)   # 识别后等待触发停车时间
        self.declare_parameter('image_height_px', 480)


        # Class labels
        self.declare_parameter('obstacle_labels', ['red_cone', 'red_zone'])
        self.declare_parameter('ignore_labels', ['num_1','num_2','num_3','num_4','num_5','num_6','num_7','num_8'])

        # --- 读取参数 ---
        in_topic = self.get_parameter('input_cmd_vel').get_parameter_value().string_value
        out_topic = self.get_parameter('output_cmd_vel').get_parameter_value().string_value
        stop_topic = self.get_parameter('stop_line_topic').get_parameter_value().string_value
        obstacle_topic = self.get_parameter('obstacle_topic').get_parameter_value().string_value
        # det_topic = self.get_parameter('detections_topic').get_parameter_value().string_value

        self.max_linear_clear = float(self.get_parameter('max_linear_clear').get_parameter_value().double_value)
        self.min_linear = float(self.get_parameter('min_linear').get_parameter_value().double_value)
        self.stop_line_hold_time = float(self.get_parameter('stop_line_hold_time').get_parameter_value().double_value)
        self.stop_line_trigger_time = float(self.get_parameter('stop_line_trigger_time').get_parameter_value().double_value)

        self.image_height_px = int(self.get_parameter('image_height_px').get_parameter_value().integer_value)
        
        self.obstacle_labels = set([str(x) for x in self.get_parameter('obstacle_labels').get_parameter_value().string_array_value])
        self.ignore_labels = set([str(x) for x in self.get_parameter('ignore_labels').get_parameter_value().string_array_value])

        # 状态机变量
        self.current_state = RobotState.CLEAR
        self.state_start_time: Optional[Time] = None
        self.last_stop_time: Optional[Time] = None   # 最近一次进入 HOLDING（停车完成）时的时间，作为 cooldown 基准
        self.latest_stopline_seen_time: Optional[Time] = None  # 最近一次收到 stop_line True 的时间
        self.stop_line_active = False   # 当前是否检测到 stop line (瞬时)

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

        # stop line: expect std_msgs/Bool topic (True when detected)
        if stop_topic:
            self.stop_sub = self.create_subscription(Bool, stop_topic, self.on_stopline, det_qos)
        else:
            self.stop_sub = None

        # optional obstacle distance topic
        if obstacle_topic:
            self.obstacle_sub = self.create_subscription(ObstacleInfo, obstacle_topic, self.on_obstacle, det_qos)
        else:
            self.obstacle_sub = None

        # optional vision detections
        # if det_topic:
        #     self.det_sub = self.create_subscription(Detection2DArray, det_topic, self.on_detections, det_qos)
        # else:
        #     self.det_sub = None

        self.get_logger().info(f'CmdVelLimiter started. in={in_topic} out={out_topic} stop_topic={stop_topic or "disabled"}')

    # -----------------------------------------
    # 回调：停车线 topic（std_msgs/Bool）
    def on_stopline(self, msg: Bool) -> None:
        now = self.get_clock().now()
        if msg.data:
            # 首次看到 True，记录时间（用于 trigger_time ）
            if self.latest_stopline_seen_time is None:
                self.latest_stopline_seen_time = now
            # 标记为当前检测到停车线（瞬时）
            self.stop_line_active = True
        else:
            # 看到 False，清除检测（reset）
            self.latest_stopline_seen_time = None
            self.stop_line_active = False

        # 每次 stop topic 更新都调用状态机（带当前检测情况）
        # 我们把 saw_obstacle=False, max_obstacle_height_px=0 (由视觉模块决定)
        self.update_state_machine(saw_obstacle=False, max_obstacle_height_px=0.0, now=now)

    def on_obstacle(self, msg: ObstacleInfo) -> None:
        self.obstacle_info = msg

    # def on_detections(self, msg: Detection2DArray) -> None:
    #     # 可选：从 vision detections 中判断是否看到障碍并计算 max_obstacle_height_px
    #     now = self.get_clock().now()
    #     saw_obstacle = False
    #     max_h = 0.0
    #     for det in msg.detections:
    #         if not det.results:
    #             continue
    #         hyp = max(det.results, key=lambda h: getattr(h.hypothesis, 'score', 0.0))
    #         label = str(hyp.hypothesis.class_id)
    #         if label in self.ignore_labels:
    #             continue
    #         if label in self.obstacle_labels:
    #             saw_obstacle = True
    #             try:
    #                 box_h = float(det.bbox.size_y)
    #                 if box_h > max_h:
    #                     max_h = box_h
    #             except Exception:
    #                 pass

    #     self.update_state_machine(saw_obstacle=saw_obstacle, max_obstacle_height_px=max_h, now=now)

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
        # 2) 检查障碍触发条件
        obstacle_triggered = False
        


        if self.current_state == RobotState.CLEAR:
            if stop_line_triggered:
                self.get_logger().info('Stop line triggered -> STOPPING')
                self.current_state = RobotState.STOPPING
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

    # cmd_vel 回调：根据状态应用限速并发布
    def on_cmd_vel(self, msg: Twist) -> None:
        now = self.get_clock().now()
        if self.current_state == RobotState.CLEAR:
            desired_cap = self.max_linear_clear
        elif self.current_state == RobotState.STOPPING:
            desired_cap = self.min_linear
        elif self.current_state == RobotState.HOLDING:
            desired_cap = self.min_linear
        else:
            desired_cap = self.max_linear_clear

        scaled = Twist()
        lin_x = msg.linear.x
        lin_y = msg.linear.y
        lin_z = msg.linear.z
        lin_magnitude = math.hypot(lin_x, lin_y)

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
            # 检查是否接近障碍物，如果接近调整角速度
            # if  hasattr(self, 'obstacle_info') and self.obstacle_info.bottom:
            #     turn_adjust = 0.1
            #     if(self.obstacle_info.left):    # 障碍物在左侧
            #         # 增加右转角速度
            #         scaled.angular.z += turn_adjust
            #     elif(self.obstacle_info.right):
            #         # 增加左转角速度
            #         scaled.angular.z -= turn_adjust
                

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
