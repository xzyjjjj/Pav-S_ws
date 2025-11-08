# ...existing code...
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

# 尝试导入 nav2_msgs/Costmap（可能不存在）

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class Checker(Node):
    def __init__(self):
        super().__init__('checker')
        self.subscription_occ = self.create_subscription(
            OccupancyGrid,
            '/processed_costmap',
            self.cb_occupancy,
            10)

    def cb_occupancy(self, msg):
        self.get_logger().info(f'Received OccupancyGrid: {msg.info}')

def main(args=None):
    rclpy.init(args=args)
    node = Checker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
# ...existing code...