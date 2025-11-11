# bev_obstacle_detector/mission_manager.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

class MissionManager(Node):
    
    STATE_IDLE = 0
    STATE_NAV_FINAL = 1
    STATE_NAV_BONUS = 2

    def __init__(self):
        super().__init__('mission_manager')

        # --- 1. 声明和获取参数 ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('final_goal_topic', '/rviz_final_goal'),
                ('bonus_goal_topic', '/bonus_goals'),
                ('nav_action_server', '/navigate_to_pose'),
                ('final_goal_value', -1)
            ]
        )
        final_goal_topic = self.get_parameter('final_goal_topic').value
        bonus_goal_topic = self.get_parameter('bonus_goal_topic').value
        nav_action_server = self.get_parameter('nav_action_server').value
        self.FINAL_GOAL_VALUE = self.get_parameter('final_goal_value').value

        # --- 2. 内部状态 ---
        self.state = self.STATE_IDLE
        self.final_goal_pose = None         # 存储 RViz 设置的终点
        self.current_active_goal_handle = None # 存储当前 Nav2 正在执行的目标句柄
        self.current_active_goal_value = self.FINAL_GOAL_VALUE
        self.current_active_goal_type = None # 'FINAL' or 'BONUS'

        # --- 3. ROS 接口 ---
        # (Input) 订阅 RViz 发布的终点
        self.create_subscription(
            PoseStamped,
            final_goal_topic,
            self.rviz_final_goal_callback,
            10
        )
        
        # (Input) 订阅 YOLO 识别的加分点
        self.create_subscription(
            MarkerArray,
            bonus_goal_topic,
            self.bonus_goals_callback,
            10
        )
        
        # (Output) Nav2 动作客户端
        self.nav_to_pose_client = ActionClient(
            self, 
            NavigateToPose, 
            nav_action_server
        )

        self.get_logger().info("Mission Manager 已启动。")
        self.get_logger().info("请在 RViz 中设置终点 (发布到 /rviz_final_goal)")

    # --- 核心回调 ---

    def rviz_final_goal_callback(self, msg: PoseStamped):
        """(输入1) 收到 RViz 终点目标"""
        # if self.state != self.STATE_IDLE:
        #      self.get_logger().warn("比赛已在进行中，忽略新的终点设置。")
        #      return
             
        self.get_logger().info("收到最终目标！开始导航...")
        self.final_goal_pose = msg
        
        # 立即将终点设为第一个目标
        self.send_new_goal(self.final_goal_pose, self.FINAL_GOAL_VALUE, 'FINAL')
        self.state = self.STATE_NAV_FINAL

    def bonus_goals_callback(self, msg: MarkerArray):
        """(输入2) 收到 YOLO 识别的加分点"""
        if self.state == self.STATE_IDLE or not msg.markers:
            # 如果比赛还未开始 (未设置终点)，或没看到目标，则忽略
            return
        marker_list = msg.markers

        self.get_logger().info('==========================bonus_goals_callback=========================\n')
        if len(marker_list) == 0:
            self.get_logger().info('当前视野不含加分点')
            self.get_logger().info('======================================================================\n\n')
            return

        self.get_logger().info(f'当前视野包含 {len(marker_list)} 个 Marker:\n')
        # 1. 找到最有价值的 bonus
        best_marker = None
        max_value = self.FINAL_GOAL_VALUE
        num_bonus = 1
        for marker in marker_list:
            self.get_logger().info(f'加分点{num_bonus}: 价值 {marker.id} \n')
            if marker.id > max_value:
                max_value = marker.id
                best_marker = marker
        
        if best_marker is None:
            self.get_logger().info('未找到最佳加分点??????\n')
            self.get_logger().info('======================================================================\n\n')
            return # 没看到有效的加分点

        # # 2. 决策：是否值得覆盖当前目标？
        # if max_value > self.current_active_goal_value:
        #     self.get_logger().info(f"发现更高价值目标 (Value: {max_value})！覆盖当前目标 (Value: {self.current_active_goal_value})。")
            
        # 创建 PoseStamped
        bonus_pose = PoseStamped()
        bonus_pose.header.frame_id = best_marker.header.frame_id # 应该是 'map'
        bonus_pose.header.stamp = self.get_clock().now().to_msg()
        bonus_pose.pose = best_marker.pose
        
        # 3. 发送新目标
        self.send_new_goal(bonus_pose, max_value, 'BONUS')
        self.state = self.STATE_NAV_BONUS
        self.get_logger().info(f"||找到最佳加分点: 价值 {max_value} !!!!!!||")
        self.get_logger().info('======================================================================\n\n')
        
    # --- Action Client 逻辑 ---

    def send_new_goal(self, pose_stamped: PoseStamped, value: int, goal_type: str):
        """核心函数：取消旧目标，发送新目标给 Nav2"""
        
        # 1. (如果需要) 取消当前正在执行的目标
        if self.current_active_goal_handle is not None and \
           self.current_active_goal_handle.status == GoalStatus.STATUS_EXECUTING:
            
            self.get_logger().warn("正在取消旧目标...")
            self.current_active_goal_handle.cancel_goal_async()

        # 2. 准备新目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        # 3. 更新内部状态
        self.current_active_goal_value = value
        self.current_active_goal_type = goal_type
        
        # 4. 发送目标
        self.get_logger().info(f"正在发送新目标 (Type: {goal_type}, Value: {value})...")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        
        # 5. 注册“目标已接受”的回调
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        """Nav2 接受或拒绝了我们的目标"""
        self.current_active_goal_handle = future.result()
        if not self.current_active_goal_handle.accepted:
            self.get_logger().error('目标被 Nav2 拒绝！')
            return

        self.get_logger().info('目标已被 Nav2 接受。')
        
        # 注册“任务已完成”的回调
        result_future = self.current_active_goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future: Future):
        """(输入3) Nav2 任务完成（成功、失败或取消）"""
        
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.current_active_goal_type == 'FINAL':
                self.get_logger().info("***********************************")
                self.get_logger().info("！！！已成功抵达最终目标！！！")
                self.get_logger().info("***********************************")
                self.state = self.STATE_IDLE
                self.final_goal_pose = None # 任务完成
            
            elif self.current_active_goal_type == 'BONUS':
                self.get_logger().info(f"加分点 (Value: {self.current_active_goal_value}) 已收集！")
                self.get_logger().info("--- 正在恢复导航至最终目标... ---")
                
                # 恢复逻辑！立即重新发布终点
                self.send_new_goal(self.final_goal_pose, self.FINAL_GOAL_VALUE, 'FINAL')
                self.state = self.STATE_NAV_FINAL

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("当前目标被取消（可能被新目标覆盖）。")

        else: # ABORTED or UNKNOWN
            self.get_logger().error(f"导航失败 (Status: {status})！")
            if self.final_goal_pose is not None:
                self.get_logger().warn("--- 正在尝试重新导航至最终目标... ---")
                self.send_new_goal(self.final_goal_pose, self.FINAL_GOAL_VALUE, 'FINAL')
                self.state = self.STATE_NAV_FINAL

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()