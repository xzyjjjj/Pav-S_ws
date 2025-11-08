import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image # 用于接收图像
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError # 用于ROS/CV图像转换

class ProximityDetectorNode(Node):
    def __init__(self):
        super().__init__('distance_node_cv')

        # 声明参数
        self.declare_parameter('mask_file_name', '90.png')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('image_topic', '/camera/color/image_raw') 
        self.declare_parameter('proximity_topic', '/object_in_proximity')
        self.declare_parameter('viz_alpha', 0.3) # 掩码的透明度
        
        # --- 新增：可视化开关参数 ---
        self.declare_parameter('enable_vis', True) # 默认开启

        # --- 新增：HSV颜色阈值参数 ---
        # 这些是黄色的典型范围 (在OpenCV H=0-179 范围内)
        # 您极有可能需要根据实际光照条件调整这些值
        self.declare_parameter('hsv_yellow_min', [22, 95, 120])
        self.declare_parameter('hsv_yellow_max', [38, 255, 255])
        
        # --- 新增：形态学操作核 ---
        self.kernel = np.ones((1, 5), np.uint8)

        # 获取参数
        mask_file_name = self.get_parameter('mask_file_name').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        proximity_topic = self.get_parameter('proximity_topic').get_parameter_value().string_value
        self.viz_alpha = self.get_parameter('viz_alpha').get_parameter_value().double_value

        # --- 新增：获取可视化开关 ---
        self.enable_viz = self.get_parameter('enable_vis').get_parameter_value().bool_value


        # 初始化 CV Bridge 和可视化窗口
        self.bridge = CvBridge()
        
        # --- 修改：仅在开启可视化时定义窗口名称 ---
        if self.enable_viz:
            self.viz_window_name = "Proximity Detection Viz"
            self.debug_window_name = "Yellow Segmentation" # 调试窗口
            self.get_logger().info(f'Visualization enabled. Displaying windows: "{self.viz_window_name}", "{self.debug_window_name}"')
        else:
            self.get_logger().info('Visualization disabled. No OpenCV windows will be shown.')


        # 构建掩码文件的完整路径
        pkg_share_dir = get_package_share_directory('distance_detector')
        mask_path = os.path.join(pkg_share_dir, 'resource', mask_file_name)

        # 加载并处理掩码
        # self.mask 是灰度图 (0=危险, 255=安全)
        self.mask = self.load_and_process_mask(mask_path)
        
        if self.mask is not None:
            # 创建一个反转的掩码用于逻辑运算 (255=危险, 0=安全)
            self.danger_mask = cv2.bitwise_not(self.mask)

            # --- 修改：仅在开启可视化时创建彩色掩码 ---
            if self.enable_viz:
                # 为可视化创建一个彩色的掩码 (危险区=红色)
                self.viz_mask_bgr = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)
                self.viz_mask_bgr[self.mask == 0] = [0, 0, 255] # 红色
                self.viz_mask_bgr[self.mask == 255] = [0, 0, 0] # 透明 (黑色)

            # --- 修改：创建单个图像订阅 ---
            self.image_subscription = self.create_subscription(
                Image,
                image_topic,
                self.image_callback,
                10)
            
            # 创建发布器
            self.publisher_ = self.create_publisher(Bool, proximity_topic, 10)
            
            self.get_logger().info(f'Detector started. Subscribing to "{image_topic}".')
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
            
            # 调整掩码大小
            resized_mask = cv2.resize(mask, (self.image_width, self.image_height), interpolation=cv2.INTER_NEAREST)
            
            # 确保是二值的 (0 和 255)
            # 我们假设危险区是黑色(0)，安全区是白色(255)
            _, binary_mask = cv2.threshold(resized_mask, 127, 255, cv2.THRESH_BINARY)
            
            self.get_logger().info(f'Resized mask to: {self.image_width}x{self.image_height}')
            return binary_mask
        except Exception as e:
            self.get_logger().error(f'Error processing mask: {e}')
            return None

    # --- 核心图像处理回调 ---
    def image_callback(self, img_msg: Image):
        """处理传入的摄像头图像"""
        object_in_proximity = False
        
        # 1. 将ROS图像转换为OpenCV图像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return
            
        # 2. 颜色分割 (BGR -> HSV)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 从参数获取HSV阈值，并显式指定数据类型为 uint8
        hsv_min = np.array(self.get_parameter('hsv_yellow_min').get_parameter_value().integer_array_value, dtype=np.uint8)
        hsv_max = np.array(self.get_parameter('hsv_yellow_max').get_parameter_value().integer_array_value, dtype=np.uint8)
        # 应用颜色阈值
        yellow_mask = cv2.inRange(hsv_image, hsv_min, hsv_max)
        
        # # 3. 形态学操作 (去噪)
        # # 开运算 (Erode -> Dilate) 去除小的噪点
        # mask_open = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, self.kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, self.kernel)
        # # 闭运算 (Dilate -> Erode) 填充物体内部的小洞
        # cleaned_mask = cv2.morphologyEx(mask_open, cv2.MORPH_CLOSE, self.kernel)

        # 4. 核心逻辑：检查重叠
        # cleaned_mask (255=黄色)
        # self.danger_mask (255=危险区)
        # 按位与操作：只有在 同一个像素 既是黄色(255) 又是 危险区(255) 时，结果才是 255
        intersection = cv2.bitwise_and(yellow_mask, self.danger_mask)

        # 检查重叠区域是否有任何像素
        if np.any(intersection):
            object_in_proximity = True
            # --- 修改：将日志记录移出可视化模块，确保始终记录 ---
            self.get_logger().warn('Yellow object detected in 0.8m proximity zone!')
            
        # 5. 发布结果
        pub_msg = Bool()
        pub_msg.data = object_in_proximity
        self.publisher_.publish(pub_msg)

        # --- 修改：步骤 6 和 7 仅在开启可视化时执行 ---
        if self.enable_viz:
            # 6. 准备可视化帧
            viz_frame = cv_image.copy()
            
            # 绘制半透明的危险区域 (红色)
            beta = 1.0 - self.viz_alpha
            blended_frame = cv2.addWeighted(viz_frame, beta, self.viz_mask_bgr, self.viz_alpha, 0.0)

            # 查找检测到的黄色物体的轮廓
            # contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 绘制轮廓 (黄色)
            cv2.drawContours(blended_frame, contours, -1, (0, 255, 255), 2)

            # 如果在危险区，显示一个大警告
            if object_in_proximity:
                # 日志已在上面记录
                cv2.putText(blended_frame, "PROXIMITY ALERT", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)

            # 7. 显示图像
            cv2.imshow(self.viz_window_name, blended_frame)
            # cv2.imshow(self.debug_window_name, cleaned_mask) # 显示二值化掩码以供调试
            cv2.imshow(self.debug_window_name, yellow_mask) # 显示二值化掩码以供调试
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    
    # --- 修改：增强主函数鲁棒性 ---
    node = None
    try:
        node = ProximityDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('KeyboardInterrupt detected, shutting down.')
    except Exception as e:
        # 捕获初始化或spin过程中的其他错误
        if node:
            node.get_logger().error(f'Unhandled exception: {e}')
        else:
            print(f'Failed to initialize node or unhandled exception: {e}')
    finally:
        # --- 修改：仅在节点成功初始化并开启可视化时才关闭窗口 ---
        if node is not None and hasattr(node, 'enable_viz') and node.enable_viz:
            cv2.destroyAllWindows()
            
        if node is not None:
            node.destroy_node()
            
        rclpy.shutdown()

if __name__ == '__main__':
    main()