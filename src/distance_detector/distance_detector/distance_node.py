import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Image # 新增：用于接收图像
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import message_filters # 新增：用于同步话题
from cv_bridge import CvBridge, CvBridgeError # 新增：用于ROS/CV图像转换

class ProximityDetectorNode(Node):
    def __init__(self):
        super().__init__('distance_node')

        # 声明参数
        self.declare_parameter('mask_file_name', 'saved_binary_image_plt.png')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('detection_topic', '/yolo_detections')
        # 新增：声明图像话题参数
        self.declare_parameter('image_topic', '/rgb_img') # 假设YOLO也在订阅这个话题
        self.declare_parameter('proximity_topic', '/object_in_proximity')
        self.declare_parameter('viz_alpha', 0.3) # 掩码的透明度

        # 获取参数
        mask_file_name = self.get_parameter('mask_file_name').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        proximity_topic = self.get_parameter('proximity_topic').get_parameter_value().string_value
        self.viz_alpha = self.get_parameter('viz_alpha').get_parameter_value().double_value

        # 新增：初始化 CV Bridge 和可视化窗口名
        self.bridge = CvBridge()
        self.viz_window_name = "Proximity Detection Viz"

        # 构建掩码文件的完整路径
        pkg_share_dir = get_package_share_directory('distance_detector')
        mask_path = os.path.join(pkg_share_dir, 'resource', mask_file_name)

        # 加载并处理掩码
        # self.mask 现在存储用于逻辑判断的 灰度掩码 (0=危险, 255=安全)
        self.mask = self.load_and_process_mask(mask_path)
        
        if self.mask is not None:
            # 新增：为可视化创建一个彩色的掩码
            # 将危险区域(0)设为红色[0,0,255]，安全区域(255)设为黑色[0,0,0]
            self.viz_mask_bgr = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)
            self.viz_mask_bgr[self.mask == 0] = [0, 0, 255] # 危险区 (红色)
            self.viz_mask_bgr[self.mask == 255] = [0, 0, 0] # 安全区 (透明)

            # --- 修改：设置消息同步 ---
            # 删除了旧的 self.subscription
            # 1. 创建两个订阅器
            self.image_sub = message_filters.Subscriber(self, Image, image_topic)
            self.detection_sub = message_filters.Subscriber(self, Detection2DArray, detection_topic)

            # 2. 创建同步器 ( approximate=True 允许轻微的时间戳差异)
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.detection_sub], 
                queue_size=10, 
                slop=0.1 # 允许0.1秒的时间戳差异
            )
            
            # 3. 注册统一的回调函数
            self.ts.registerCallback(self.synchronized_callback)
            # ------------------------------
            
            # 创建发布器
            self.publisher_ = self.create_publisher(Bool, proximity_topic, 10)
            
            self.get_logger().info(f'Detector started. Synchronizing "{image_topic}" and "{detection_topic}".')
            self.get_logger().info(f'Publishing proximity status to "{proximity_topic}".')
        else:
            self.get_logger().error(f'Failed to load mask file from: {mask_path}. Shutting down.')
            rclpy.shutdown()

    def load_and_process_mask(self, path):
        """加载掩码图像，调整大小并进行二值化处理"""
        try:
            mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if mask is None:
                self.get_logger().error(f'Mask file not found at {path}')
                return None
            
            self.get_logger().info(f'Original mask dimensions: {mask.shape[1]}x{mask.shape[0]}')

            # 调整掩码大小
            resized_mask = cv2.resize(mask, (self.image_width, self.image_height), interpolation=cv2.INTER_NEAREST)
            
            # 确保是二值的 (0 和 255)
            # 我们假设危险区是黑色(0)，安全区是白色(255)
            _, binary_mask = cv2.threshold(resized_mask, 127, 255, cv2.THRESH_BINARY)
            
            self.get_logger().info(f'Resized mask to: {self.image_width}x{self.image_height}')
            
            return binary_mask # 返回灰度掩码用于逻辑
        except Exception as e:
            self.get_logger().error(f'Error processing mask: {e}')
            return None

    # --- 新的回调函数 ---
    def synchronized_callback(self, img_msg: Image, det_msg: Detection2DArray):
        """
        处理同步后的图像和检测结果
        """
        object_in_proximity = False
        
        # 1. 将ROS图像转换为OpenCV图像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return
            
        # 2. 准备一个用于绘制的图像副本
        viz_frame = cv_image.copy()

        # 3. 遍历所有检测
        for detection in det_msg.detections:
            # 提取坐标
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            width = detection.bbox.size_x
            height = detection.bbox.size_y
            x1 = int(center_x - width / 2)
            y1 = int(center_y - height / 2)
            x2 = int(center_x + width / 2)
            y2 = int(center_y + height / 2)

            # 裁剪坐标
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(self.image_width, x2)
            y2 = min(self.image_height, y2)

            if x1 >= x2 or y1 >= y2:
                continue

            # 4. 核心逻辑：检查是否在危险区
            mask_roi = self.mask[y1:y2, x1:x2]
            is_in_zone = False
            if np.any(mask_roi == 0): # 检查是否碰到了黑色(0)像素
                object_in_proximity = True
                is_in_zone = True

            # 5. 绘制检测框 (危险区=红色, 安全区=绿色)
            color = (0, 0, 255) if is_in_zone else (0, 255, 0)
            cv2.rectangle(viz_frame, (x1, y1), (x2, y2), color, 2)
            
            # (可选) 绘制标签
            if detection.results:
                obj_id = detection.results[0].hypothesis.class_id
                score = detection.results[0].hypothesis.score
                cv2.putText(viz_frame, f'{obj_id}: {score:.2f}', (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 6. 发布结果
        pub_msg = Bool()
        pub_msg.data = object_in_proximity
        self.publisher_.publish(pub_msg)

        # 7. 创建可视化
        # 将彩色的掩码 (viz_mask_bgr) 与 绘制了检测框的图像 (viz_frame) 混合
        # viz_mask_bgr中，安全区是[0,0,0]，所以addWeighted不会影响安全区
        beta = 1.0 - self.viz_alpha
        blended_frame = cv2.addWeighted(viz_frame, beta, self.viz_mask_bgr, self.viz_alpha, 0.0)

        # 8. 显示图像
        cv2.imshow(self.viz_window_name, blended_frame)
        cv2.waitKey(1) # 必须有waitKey(1)来刷新OpenCV窗口


def main(args=None):
    rclpy.init(args=args)
    node = ProximityDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 新增：关闭OpenCV窗口
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()