import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
import cv2
import numpy as np

IMG_PATH = '/Pav-S_ws/src/costmap_process/assets/map/map_origin.jpg'
TARGET_RGB = (220, 20, 60)
SUB_TOPIC = '/global_costmap/costmap'  # 明确订阅话题
PUB_TOPIC = '/processed_costmap'           # 明确发布话题
IMG_TOPIC = '/debug_drawn_img'           # 新增订阅 IMAGE 话题

# def find_rgb_points(img, rgb):
#     bgr = (rgb[2], rgb[1], rgb[0])
#     mask = np.all(img == bgr, axis=2)
#     ys, xs = np.where(mask)
#     return list(zip(xs, ys))
def find_rgb_points(img, rgb, threshold=3):
    bgr = np.array([rgb[2], rgb[1], rgb[0]], dtype=np.int16)
    img_bgr = img.astype(np.int16)
    diff = img_bgr - bgr
    dist2 = np.sum(diff * diff, axis=2)
    mask = dist2 <= (threshold * threshold)
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
        self.pub = self.create_publisher(OccupancyGrid, PUB_TOPIC, 10)
        self.costmap = None
        self.info = MapMetaData(
            map_load_time=Time(sec=0, nanosec=0),
            resolution=0.009999999776482582,
            width=500,
            height=300,
            origin=Pose(
                position=Point(x=0.0, y=-0.03, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )
        )
        self.processed = False
        self.img_data = None  # 保存收到的 IMAGE 数据

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)


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
        if self.costmap is None:
            self.get_logger().info('收到首次costmap，初始化...')
            self.costmap = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        else:
            pass
        self.process_and_publish()

    # def process_and_publish(self):
    #     print("in process_and_publish")
    #     # 优先使用收到的 IMAGE 数据，否则用本地文件
    #     if self.img_data is not None:
    #         img = self.img_data
    #         self.get_logger().info('使用 debug_drawn_img 作为处理图片')
    #         # 在这里弹窗显示一下 img
    #         # ==================== 可视化调试 ==================== 
    #         cv2.imshow("received debug_drawn_img", img)
    #         cv2.waitKey(0)
    #         cv2.destroyWindow("received debug_drawn_img")
    #         # ================================================
    #     else: # TODO: 这里得一直等到接收到图片
    #         img = cv2.imread(IMG_PATH)
    #         self.get_logger().info(f'使用本地图片作为处理图片: {IMG_PATH}') 
    #         if img is None:
    #             self.get_logger().error(f'图片读取失败: {IMG_PATH}')
    #             return
    #     h, w = img.shape[:2]
    #     points = find_rgb_points(img, TARGET_RGB)
    #     self.get_logger().info(f'找到 {len(points)} 个目标点')

    #     # ==================== 可视化调试 ==================== 
    #     # 构造可视化图像：红色点为白色，其他为黑色
    #     vis_img = np.zeros((h, w, 3), dtype=np.uint8)
    #     for x, y in points:
    #         vis_img[y, x] = (255, 255, 255)  # 白色

    #     # 弹窗显示
    #     cv2.imshow("Red Points Visualization", vis_img)
    #     cv2.waitKey(0)
    #     cv2.destroyWindow("Red Points Visualization")

    #     # ================================================

    #     fliped_points = flip_points(points, w)
    #     dilated_points = dilate_points(fliped_points, img.shape, k=1)
    #     self.get_logger().info(f'膨胀后点数: {len(dilated_points)}')
    #     for x, y in dilated_points:
    #         if 0 <= x < self.costmap.shape[1] and 0 <= y < self.costmap.shape[0]:
    #             self.costmap[y, x] = 100
    #     out = OccupancyGrid()
    #     out.header.frame_id = 'map'
    #     out.info = self.info
    #     out.data = self.costmap.flatten().tolist()
    #     self.pub.publish(out)
    #     self.get_logger().info('已发布 /processed_costmap')

    def process_and_publish(self):
        print("in process_and_publish")
        # 优先使用收到的 IMAGE 数据，否则用本地文件
        if self.img_data is not None:
            img = self.img_data
            self.get_logger().info('使用 debug_drawn_img 作为处理图片')
        else:
            img = cv2.imread(IMG_PATH)
            self.get_logger().info(f'使用本地图片作为处理图片: {IMG_PATH}') 
            if img is None:
                self.get_logger().error(f'图片读取失败: {IMG_PATH}')
                return
        h, w = img.shape[:2]
        # 只判定完全等于目标RGB的点
        mask = (img[:,:,2] == TARGET_RGB[0]) & (img[:,:,1] == TARGET_RGB[1]) & (img[:,:,0] == TARGET_RGB[2])
        vis_img = np.zeros((h, w, 3), dtype=np.uint8)
        vis_img[mask] = (255, 255, 255)  # 白色
        # OUT_PATH = '/Pav-S_ws/src/costmap_process/costmap_pub/red_points_mask.png'
        cv2.imwrite(OUT_PATH, vis_img)
        print(f'已保存结果图片到: {OUT_PATH}')
        # 弹窗显示
        cv2.imshow("Red Points Visualization", vis_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # 上下翻转
        mask_flip = np.flipud(mask)

        # 获取 global_costmap 并叠加
        if self.costmap is None:
            self.get_logger().error("global_costmap 尚未初始化，无法叠加和发布")
            return
        costmap = self.costmap.copy()
        # 叠加红色点（白色区域）到 costmap，赋值为障碍
        for y in range(h):
            for x in range(w):
                if mask_flip[y, x]:
                    costmap[y, x] = 100  # 叠加障碍

        # 发布 OccupancyGrid
        out = OccupancyGrid()
        out.header.frame_id = 'map'
        out.info = self.info
        out.data = costmap.flatten().tolist()
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