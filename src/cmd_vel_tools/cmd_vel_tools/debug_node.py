#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class TestPublisher(Node):
    def __init__(self):
        super().__init__('debug_node')

        # 发布者
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav2', 10)
        self.stop_pub = self.create_publisher(Bool, '/object_in_proximity', 10)

        # 定时器：10 Hz
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 状态
        self.start_time = self.get_clock().now()
        self.get_logger().info("Test node started, publishing cmd_vel and /stop_line")

    def timer_callback(self):
        # 当前时间（秒）
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # --- 发布 cmd_vel ---
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        # --- 控制 stop_line ---
        stop = Bool()
        # 10~13s、30~33s 置 true
        if (10 <= elapsed <= 13) or (30 <= elapsed <= 33):
            stop.data = True
        else:
            stop.data = False
        self.stop_pub.publish(stop)

        # --- 输出状态 ---
        if int(elapsed) % 2 == 0:  # 每 5 秒打印一次
            self.get_logger().info(f"Time: {elapsed:.1f}s | stop_line={stop.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
