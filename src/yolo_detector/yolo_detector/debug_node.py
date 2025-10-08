import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32
from enum import Enum
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from vision_msgs.msg import Pose2D, Point2D

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')

        # 发布假速度指令，用于测试
        self.cmd_test_pub = self.create_publisher(Twist, '/cmd_vel_nav2', 10)

        # 订阅限速后的速度指令
        self.cmd_debug_sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_debug, 10)

        # 定时发布假速度指令
        self.timer = self.create_timer(0.2, self.publish_fake_cmd)  # 每秒发布一次

        # -----------------------------------------
        # 以下代码为 CmdVelLimiter 的一部分

    def publish_fake_cmd(self):
        """发布假速度指令，用于测试"""
        msg = Twist()
        msg.linear.x = 0.5  # 模拟前进速度
        msg.linear.y = 0.0
        msg.angular.z = 0.2
        self.cmd_test_pub.publish(msg)
        self.get_logger().info("Published fake /cmd_vel_nav2")

    def on_cmd_debug(self, msg: Twist):
        """调试输出限速后的速度"""
        self.get_logger().info(
            f"Output /cmd_vel: linear_x={msg.linear.x:.2f}, linear_y={msg.linear.y:.2f}, \
                               angular={msg.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  
    main()