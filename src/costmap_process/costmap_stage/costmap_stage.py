import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# 如果用tf2_ros，需要安装tf2_ros并导入相关包

class ConfigPublisher(Node):
    def __init__(self):
        super().__init__('config_publisher')
        self.publisher = self.create_publisher(String, '/costmap_num_config', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # 这里假设你能获取到 x，实际应通过 tf2 查询 map->body 的变换
        x, y = self.get_body_xy()  # 你需要实现这个方法
        msg = String()
        if 3.98 < x < 5 and 0 < y < 1.5:
            detect_valid = 1
            stage = 1
            msg.data = '1,1'  # detect_valid=1, stage=1
        elif 0 < x < 1.3 and 1.5 < y < 3:
             detect_valid = 1
             stage = 2
             msg.data = '1,2'  # detect_valid=1, stage=2
        else:
            detect_valid = 0
            stage = 0
            msg.data = '0,0'  # detect_valid=0, stage=0
        self.publisher.publish(msg)
        self.get_logger().info(f'x={x}, y={y}')
        self.get_logger().info(f'发布: detect_valid={detect_valid}, stage={stage} (x={x}, y={y})')

    def get_body_xy(self):
        try:
            # 查询 map 到 base_link 的变换（可根据实际机器人改成 camera_link）
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            self.get_logger().info(f"map->base 变换: x={x:.3f}, y={y:.3f}")
            return x, y
        except Exception as e:
            self.get_logger().warn(f"TF查询失败: {e}")
            return 0.0, 0.0  # 查询失败时返回默认值

def main(args=None):
    rclpy.init(args=args)
    node = ConfigPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()