#!/usr/bin/env python3
# coding=utf-8

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32
from enum import Enum
from std_srvs.srv import SetBool



class RobotState(Enum):
    CLEAR = "CLEAR"           # 无危险，全速
    CAUTIOUS = "CAUTIOUS"     # 有障碍或红区，限速
    STOPPING = "STOPPING"     # 停车线触发，减速中
    HOLDING = "HOLDING"       # 保持停止


class CmdVelLimiter(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_limiter')

        # Topics
        self.declare_parameter('input_cmd_vel', '/cmd_vel')  # match existing topic
        self.declare_parameter('output_cmd_vel', '/cmd_vel')
        self.declare_parameter('detections_topic', '/yolo_detections')
        self.declare_parameter('stop_line_distance_topic', '')  # 停车线距离话题(Float32: m)
        self.declare_parameter('obstacle_distance_topic', '')  # 障碍距离话题(Float32: m)

        # Speed caps (m/s, rad/s)
        self.declare_parameter('max_linear_clear', 0.6)
        self.declare_parameter('cautious_linear', 0.25)
        self.declare_parameter('min_linear', 0.0)
        self.declare_parameter('max_angular', 1.0)

        # 停车线检测的距离/框大小触发参数
        self.declare_parameter('decel_duration', 1.0)
        self.declare_parameter('stop_line_hold_time', 2.0)
        self.declare_parameter('stop_line_trigger_distance_m', 0.4)  # 触发距离，默认0.4m
        self.declare_parameter('image_height_px', 480)
        self.declare_parameter('stop_line_height_ratio_trigger', 0.25)  # 无距离时按照框高比例触发
        
        # 障碍检测的距离/框大小触发参数
        self.declare_parameter('obstacle_trigger_distance_m', 0.6)  # 障碍触发距离
        self.declare_parameter('obstacle_height_ratio_trigger', 0.35)  # 障碍框高比例触发阈值

        # Class labels
        self.declare_parameter('stop_line_labels', ['yellow_stop_line'])
        self.declare_parameter('obstacle_labels', ['red_cone', 'red_zone'])
        self.declare_parameter('ignore_labels', ['bonus'])

        # Params fetch
        in_topic = self.get_parameter('input_cmd_vel').get_parameter_value().string_value
        out_topic = self.get_parameter('output_cmd_vel').get_parameter_value().string_value
        det_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        dist_topic = self.get_parameter('stop_line_distance_topic').get_parameter_value().string_value
        obs_dist_topic = self.get_parameter('obstacle_distance_topic').get_parameter_value().string_value

        self.max_linear_clear = float(self.get_parameter('max_linear_clear').get_parameter_value().double_value)
        self.cautious_linear = float(self.get_parameter('cautious_linear').get_parameter_value().double_value)
        self.min_linear = float(self.get_parameter('min_linear').get_parameter_value().double_value)
        self.max_angular = float(self.get_parameter('max_angular').get_parameter_value().double_value)
        self.decel_duration = float(self.get_parameter('decel_duration').get_parameter_value().double_value)
        self.stop_line_hold_time = float(self.get_parameter('stop_line_hold_time').get_parameter_value().double_value)

        self.stop_line_labels = set([str(x) for x in self.get_parameter('stop_line_labels').get_parameter_value().string_array_value])
        self.obstacle_labels = set([str(x) for x in self.get_parameter('obstacle_labels').get_parameter_value().string_array_value])
        self.ignore_labels = set([str(x) for x in self.get_parameter('ignore_labels').get_parameter_value().string_array_value])
        self.stop_line_trigger_distance_m = float(self.get_parameter('stop_line_trigger_distance_m').get_parameter_value().double_value)
        self.image_height_px = int(self.get_parameter('image_height_px').get_parameter_value().integer_value)
        self.stop_line_height_ratio_trigger = float(self.get_parameter('stop_line_height_ratio_trigger').get_parameter_value().double_value)
        self.obstacle_trigger_distance_m = float(self.get_parameter('obstacle_trigger_distance_m').get_parameter_value().double_value)
        self.obstacle_height_ratio_trigger = float(self.get_parameter('obstacle_height_ratio_trigger').get_parameter_value().double_value)

        # State machine
        self.current_state = RobotState.CLEAR
        self.state_start_time = None
        self.latest_stopline_distance_m = None
        self.latest_stopline_distance_stamp = None
        self.latest_obstacle_distance_m = None
        self.latest_obstacle_distance_stamp = None

        # QoS
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        det_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        # IO
        self.cmd_sub = self.create_subscription(Twist, in_topic, self.on_cmd_vel, cmd_qos)
        self.cmd_pub = self.create_publisher(Twist, out_topic, cmd_qos)
        self.det_sub = self.create_subscription(Detection2DArray, det_topic, self.on_detections, det_qos)
        if dist_topic:
            self.dist_sub = self.create_subscription(Float32, dist_topic, self.on_stopline_distance, det_qos)
        if obs_dist_topic:
            self.obs_dist_sub = self.create_subscription(Float32, obs_dist_topic, self.on_obstacle_distance, det_qos)

        # --------------------
        self.detection_pub = self.create_publisher(Detection2DArray, "/yolo_detections", 10)
        # --------------------

        self.get_logger().info('CmdVelLimiter started. in=%s out=%s det=%s stop_dist=%s obs_dist=%s' % 
                              (in_topic, out_topic, det_topic, (dist_topic or 'disabled'), (obs_dist_topic or 'disabled')))

    # -----------------------------
        self.debug_service_ = self.create_service(SetBool, 'debug_service', self.on_force_stop_line)
        self.force_stop_line_enabled = False

    def on_force_stop_line(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.force_stop_line_enabled = request.data
        response.success = True
        response.message = f"force_stop_line set to {request.data}"
        self.get_logger().info(response.message)
        return response
    # -----------------------------


    def on_stopline_distance(self, msg: Float32) -> None:
        self.latest_stopline_distance_m = float(msg.data)
        self.latest_stopline_distance_stamp = self.get_clock().now()

    def on_obstacle_distance(self, msg: Float32) -> None:
        self.latest_obstacle_distance_m = float(msg.data)
        self.latest_obstacle_distance_stamp = self.get_clock().now()

    def on_detections(self, msg: Detection2DArray) -> None:
        now = self.get_clock().now()
        saw_stop_line = False
        saw_obstacle = False

        max_stopline_height_px = 0.0
        max_obstacle_height_px = 0.0
        for det in msg.detections:
            if not det.results:
                continue
            hyp = max(det.results, key=lambda h: getattr(h.hypothesis, 'score', 0.0))
            label = str(hyp.hypothesis.class_id)
            if label in self.ignore_labels:
                continue
            if label in self.stop_line_labels:
                saw_stop_line = True
                # 跟踪最高的停车线框高度
                try:
                    box_h = float(det.bbox.size_y)
                    if box_h > max_stopline_height_px:
                        max_stopline_height_px = box_h
                except Exception:
                    pass
            if label in self.obstacle_labels:
                saw_obstacle = True
                # 跟踪最高的障碍框高度
                try:
                    box_h = float(det.bbox.size_y)
                    if box_h > max_obstacle_height_px:
                        max_obstacle_height_px = box_h
                except Exception:
                    pass

        # 状态机更新逻辑
        # self.update_state_machine(saw_stop_line, saw_obstacle, max_stopline_height_px, max_obstacle_height_px, now)
        self.get_logger().info(f"force_stop_line_enabled set, now calling update_state_machine manually")
        self.update_state_machine(False, False, 0, 0, self.get_clock().now())

    def update_state_machine(self, saw_stop_line, saw_obstacle, max_stopline_height_px, max_obstacle_height_px, now):
        """状态机更新逻辑"""
        # 检查停车线触发条件
        stop_line_triggered = False
        # -----------------------------
        if self.force_stop_line_enabled:
            stop_line_triggered = True
        # -----------------------------
        elif saw_stop_line:
            if self.latest_stopline_distance_m is not None:
                stop_line_triggered = (self.latest_stopline_distance_m <= self.stop_line_trigger_distance_m)
            else:
                if self.image_height_px > 0:
                    height_ratio = max_stopline_height_px / float(self.image_height_px)
                    stop_line_triggered = (height_ratio >= self.stop_line_height_ratio_trigger)

        # 检查障碍触发条件
        obstacle_triggered = False
        if saw_obstacle:
            if self.latest_obstacle_distance_m is not None:
                obstacle_triggered = (self.latest_obstacle_distance_m <= self.obstacle_trigger_distance_m)
            else:
                if self.image_height_px > 0:
                    height_ratio = max_obstacle_height_px / float(self.image_height_px)
                    obstacle_triggered = (height_ratio >= self.obstacle_height_ratio_trigger)

        # 状态转换逻辑
        if self.current_state == RobotState.CLEAR:
            if stop_line_triggered:
                self.current_state = RobotState.STOPPING
                self.state_start_time = now
            elif obstacle_triggered:
                self.current_state = RobotState.CAUTIOUS
                self.state_start_time = now

        elif self.current_state == RobotState.CAUTIOUS:
            if stop_line_triggered:
                self.current_state = RobotState.STOPPING
                self.state_start_time = now
            elif not obstacle_triggered:
                self.current_state = RobotState.CLEAR
                self.state_start_time = now

        elif self.current_state == RobotState.STOPPING:
            dt = (now - self.state_start_time).nanoseconds / 1e9
            if dt >= self.decel_duration:
                self.current_state = RobotState.HOLDING
                self.state_start_time = now

        elif self.current_state == RobotState.HOLDING:
            dt = (now - self.state_start_time).nanoseconds / 1e9
            if dt >= self.stop_line_hold_time:
                if obstacle_triggered:
                    self.current_state = RobotState.CAUTIOUS
                else:
                    self.current_state = RobotState.CLEAR
                self.state_start_time = now
        self.get_logger().info('Current state: %s' % self.current_state)


    def on_cmd_vel(self, msg: Twist) -> None:
        now = self.get_clock().now()
        
        # 根据状态机确定速度上限
        if self.current_state == RobotState.CLEAR:
            desired_cap = self.max_linear_clear
        elif self.current_state == RobotState.CAUTIOUS:
            desired_cap = self.cautious_linear
        elif self.current_state == RobotState.STOPPING:
            # 平滑减速
            self.get_logger().info('Begin to stop...')
            dt = (now - self.state_start_time).nanoseconds / 1e9
            if dt <= self.decel_duration:
                ratio = max(0.0, 1.0 - dt / self.decel_duration)
                desired_cap = max(self.min_linear, self.max_linear_clear * ratio)
            else:
                desired_cap = self.min_linear
        elif self.current_state == RobotState.HOLDING:
            desired_cap = self.min_linear
        else:
            desired_cap = self.max_linear_clear

        # 基于原cmd_vel进行速度调整：线速度缩放 + 角速度独立限制
        scaled = Twist()
        
        # 计算线速度缩放比例
        lin_x = msg.linear.x
        lin_y = msg.linear.y
        lin_z = msg.linear.z
        lin_magnitude = math.hypot(lin_x, lin_y)
        
        if lin_magnitude > 1e-6 and lin_magnitude > desired_cap:
            lin_scale = desired_cap / lin_magnitude
        else:
            lin_scale = 1.0
        
        # 线速度缩放
        scaled.linear.x = lin_x * lin_scale
        scaled.linear.y = lin_y * lin_scale
        scaled.linear.z = lin_z  # z轴保持原值
        
        # 角速度独立限制（不跟随线速度缩放）
        ang_x = msg.angular.x
        ang_y = msg.angular.y
        ang_z = msg.angular.z
        
        # 只对z轴角速度进行上限限制，保持导航的转向补偿能力
        if abs(ang_z) > self.max_angular:
            scaled.angular.z = math.copysign(self.max_angular, ang_z)
        else:
            scaled.angular.z = ang_z
            
        # x,y轴角速度保持原值（通常为0）
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