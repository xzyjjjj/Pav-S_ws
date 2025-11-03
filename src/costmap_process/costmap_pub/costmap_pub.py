import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
import cv2
import numpy as np

IMG_PATH = '/Users/xuzhenyu/Desktop/reutrn_stage/Pav-S_ws/src/costmap_process/assets/new_costmaps/draw_point_on_map_20251031_182640_505047.png'
TARGET_RGB = (220, 20, 60)
SUB_TOPIC = '/global_costmap/costmap_raw'  # 明确订阅话题
PUB_TOPIC = '/processed_costmap'           # 明确发布话题
IMG_TOPIC = '/debug_drawn_image'           # 新增订阅 IMAGE 话题

def find_rgb_points(img, rgb):
    bgr = (rgb[2], rgb[1], rgb[0])
    mask = np.all(img == bgr, axis=2)
    ys, xs = np.where(mask)
    return list(zip(xs, ys))

def flip_points(points, img_w):
    return [(img_w - 1 - x, y) for x, y in points]

def dilate_points(points, img_shape, k=3):
    dilated = set()
    h, w = img_shape[:2]
    r = k // 2
    for x, y in points:
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    dilated.add((nx, ny))
    return list(dilated)

class CostmapFliper(Node):
    def __init__(self):
        super().__init__('costmap_fliper')
        self.sub = self.create_subscription(OccupancyGrid, SUB_TOPIC, self.cb, 10)
        self.img_sub = self.create_subscription(Image, IMG_TOPIC, self.img_cb, 10)
        # 使用明确的 QoSProfile，可以与需要 TRANSIENT_LOCAL 的订阅者兼容
        qos = QoSProfile(depth=10)
        # 如果订阅方希望获得发布方的“最后一条”历史（类似 latched），
        # 需要使用 TRANSIENT_LOCAL；这里设置为 TRANSIENT_LOCAL 以兼容此类订阅者。
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, PUB_TOPIC, qos)
        # 诊断日志：记录节点启动与 QoS 配置，便于排查“没有输出”的问题
        try:
            self.get_logger().info(f'CostmapFliper started. sub:{SUB_TOPIC} img_sub:{IMG_TOPIC} pub:{PUB_TOPIC}')
            self.get_logger().info(f'Publisher QoS: durability={qos.durability}, reliability={qos.reliability}, depth={qos.depth}')
        except Exception:
            # 保证即便 logger 有问题也不抛出
            pass
        self.costmap = None
        self.info = None
        self.processed = False
        self.img_data = None  # 保存收到的 IMAGE 数据

    def img_cb(self, msg):
        # 将 ROS Image 消息转为 numpy BGR 图像
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding.lower() in ['bgr8', 'rgb8']:
                img = arr.reshape((msg.height, msg.width, 3))
                if msg.encoding.lower() == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                self.img_data = img.copy()
                self.get_logger().info('收到 debug_drawn_image，已缓存')
            else:
                self.get_logger().warn(f'不支持的图像编码: {msg.encoding}')
        except Exception as e:
            self.get_logger().error(f'图像解码失败: {e}')

    def cb(self, msg):
        # 调试日志：确认回调被触发
        self.get_logger().info('cb() called')
        if self.costmap is None:
            self.get_logger().info('收到首次costmap，初始化...')
            self.info = msg.info
            self.costmap = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            self.process_and_publish()
        else:
            # 如果已经处理过首次 costmap，可以在这里记录后续行为（当前实现忽略）
            self.get_logger().debug('收到后续 costmap，已忽略')

    def process_and_publish(self):
        # 优先使用收到的 IMAGE 数据，否则用本地文件
        if self.img_data is not None:
            img = self.img_data
            self.get_logger().info('使用 debug_drawn_image 作为处理图片')
        else:
            img = cv2.imread(IMG_PATH)
            if img is None:
                self.get_logger().error(f'图片读取失败: {IMG_PATH}')
                return
        h, w = img.shape[:2]
        points = find_rgb_points(img, TARGET_RGB)
        self.get_logger().info(f'找到 {len(points)} 个目标点')
        fliped_points = flip_points(points, w)
        dilated_points = dilate_points(fliped_points, img.shape, k=3)
        self.get_logger().info(f'膨胀后点数: {len(dilated_points)}')
        for x, y in dilated_points:
            if 0 <= x < self.costmap.shape[1] and 0 <= y < self.costmap.shape[0]:
                self.costmap[y, x] = 100
        out = OccupancyGrid()
        out.header.frame_id = 'map'
        out.info = self.info
        out.data = self.costmap.flatten().tolist()
        self.pub.publish(out)
        self.get_logger().info('已发布 /processed_costmap')

def main(args=None):
    rclpy.init(args=args)
    node = CostmapFliper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()