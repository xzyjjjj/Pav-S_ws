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
        self.declare_parameter('mask_file_name', 'mask90.png')
        self.declare_parameter('bev_mask_file_name', 'bev_mask.png')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('image_topic', '/camera/color/image_raw') 
        self.declare_parameter('proximity_topic', '/object_in_proximity')
        self.declare_parameter('red_in_bev_topic', '/object_in_bev') # 针对红色检测
        self.declare_parameter('viz_alpha', 0.3) # 掩码的透明度
        
        # --- 可视化开关参数 ---
        self.declare_parameter('enable_vis', True) # 默认开启

        # --- HSV颜色阈值参数 ---
        self.declare_parameter('hsv_yellow_min', [22, 95, 120])
        self.declare_parameter('hsv_yellow_max', [38, 255, 255])
        
        # --- 形态学操作核 ---
        # 保持了 (1, 5) 的横向核，但建议测试 (5, 5) 的方形核以获得更通用的去噪效果
        self.kernel = np.ones((1, 5), np.uint8) 

        # 获取参数
        mask_file_name = self.get_parameter('mask_file_name').get_parameter_value().string_value
        bev_mask_file_name = self.get_parameter('bev_mask_file_name').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        proximity_topic = self.get_parameter('proximity_topic').get_parameter_value().string_value
        red_in_bev_topic = self.get_parameter('red_in_bev_topic').get_parameter_value().string_value
        self.viz_alpha = self.get_parameter('viz_alpha').get_parameter_value().double_value

        # 获取可视化开关
        self.enable_viz = self.get_parameter('enable_vis').get_parameter_value().bool_value


        # 初始化 CV Bridge 和可视化窗口
        self.bridge = CvBridge()
        
        # 仅在开启可视化时定义窗口名称
        if self.enable_viz:
            self.viz_window_name = "Proximity Detection Viz"
            self.debug_window_name = "Yellow Segmentation" # 调试窗口
            self.bev_viz_window_name = "Bev Detection Viz"
            self.bev_debug_window_name = "Red Segmentation" # 调试窗口
            self.get_logger().info(f'Visualization enabled. Displaying windows: "{self.viz_window_name}", "{self.debug_window_name}"')
        else:
            self.get_logger().info('Visualization disabled. No OpenCV windows will be shown.')


        # 构建掩码文件的完整路径
        try:
            pkg_share_dir = get_package_share_directory('distance_detector')
        except Exception:
            self.get_logger().error("Could not find package 'distance_detector'. Check package name.")
            rclpy.shutdown()
            return

        mask_path = os.path.join(pkg_share_dir, 'resource', mask_file_name)
        bev_mask_path = os.path.join(pkg_share_dir, 'resource', bev_mask_file_name)

        # 加载并处理掩码
        self.mask = self.load_and_process_mask(mask_path)
        self.bev_mask = self.load_and_process_mask(bev_mask_path)
        
        # 确保基础掩码加载成功
        if self.mask is not None and self.bev_mask is not None:
            # 创建反转的掩码用于逻辑运算 (255=危险, 0=安全)
            self.danger_mask = cv2.bitwise_not(self.mask)
            # 对BEV掩码也进行反转 (假设危险区是黑色(0))
            self.bev_danger_mask = cv2.bitwise_not(self.bev_mask) 

            # 仅在开启可视化时创建彩色掩码
            if self.enable_viz:
                # 为可视化创建一个彩色的掩码 (危险区=红色)
                self.viz_mask_bgr = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)
                self.bev_viz_mask_bgr = cv2.cvtColor(self.bev_mask, cv2.COLOR_GRAY2BGR)
                self.viz_mask_bgr[self.mask == 0] = [0, 0, 255] # 红色
                self.viz_mask_bgr[self.mask == 255] = [0, 0, 0] # 透明 (黑色)
                self.bev_viz_mask_bgr[self.bev_mask == 0] = [255, 0, 0] # 蓝
                self.bev_viz_mask_bgr[self.bev_mask == 255] = [0, 0, 0] # 透明 (黑色)

            # 创建单个图像订阅
            self.image_subscription = self.create_subscription(
                Image,
                image_topic,
                self.image_callback,
                10)
            
            # 创建发布器
            self.publisher_ = self.create_publisher(Bool, proximity_topic, 10)
            # 🚨 修复: 确保 bev_publisher 使用 red_in_bev_topic
            self.bev_publisher_ = self.create_publisher(Bool, red_in_bev_topic, 10) 
            
            self.get_logger().info(f'Detector started. Subscribing to "{image_topic}".')
            self.get_logger().info(f'Publishing yellow proximity status to "{proximity_topic}".')
            self.get_logger().info(f'Publishing red BEV status to "{red_in_bev_topic}".')
        else:
            self.get_logger().error(f'Failed to load one or both mask files. Shutting down.')
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
            # 假设危险区是黑色(0)，安全区是白色(255)
            _, binary_mask = cv2.threshold(resized_mask, 127, 255, cv2.THRESH_BINARY)
            
            self.get_logger().info(f'Resized mask from {os.path.basename(path)} to: {self.image_width}x{self.image_height}')
            return binary_mask
        except Exception as e:
            self.get_logger().error(f'Error processing mask: {e}')
            return None

    # --- 核心图像处理回调 ---
    def image_callback(self, img_msg: Image):
        """处理传入的摄像头图像"""
        object_in_proximity = False
        # 🚨 修复: 确保 object_in_bev 被初始化
        object_in_bev = False       
        
        # 1. 将ROS图像转换为OpenCV图像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return
            
        # 2. 颜色分割 (BGR -> HSV)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        ########################红色检测 (BEV) ##############################
        
        # 动态V值阈值
        mean_v = np.mean(hsv_image[:, :, 2])
        v_thresh = max(50, int(mean_v * 0.5)) 

        # 红色范围 1 (0-10)
        lower_red_1 = np.array([0, 90, v_thresh])
        upper_red_1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
        
        # 红色范围 2 (170-180)
        lower_red_2 = np.array([170, 80, v_thresh])
        upper_red_2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
        
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # 形态学操作
        morph_kernel = np.ones((1, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, morph_kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, morph_kernel)

        # 核心逻辑：检查红色物体是否在 BEV 危险区
        # red_mask (255=红色)
        # self.bev_danger_mask (255=BEV危险区)
        bev_intersection = cv2.bitwise_and(red_mask, self.bev_danger_mask)
        
        if np.any(bev_intersection):
            object_in_bev = True
            self.get_logger().warn('Red object detected in BEV zone!')

        # 5. 发布 BEV 结果
        bev_pub_msg = Bool()
        bev_pub_msg.data = object_in_bev
        self.bev_publisher_.publish(bev_pub_msg)

        ########################黄色检测 (近距离) ##############################
        # 从参数获取HSV阈值，并显式指定数据类型为 uint8
        hsv_min = np.array(self.get_parameter('hsv_yellow_min').get_parameter_value().integer_array_value, dtype=np.uint8)
        hsv_max = np.array(self.get_parameter('hsv_yellow_max').get_parameter_value().integer_array_value, dtype=np.uint8)
        
        # 应用颜色阈值
        yellow_mask = cv2.inRange(hsv_image, hsv_min, hsv_max)
        
        # 形态学操作 (去噪)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, self.kernel)
        # 建议：如果需要，可以增加闭运算
        # yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, self.kernel) 

        # 4. 核心逻辑：检查重叠
        # yellow_mask (255=黄色)
        # self.danger_mask (255=近距离危险区)
        intersection = cv2.bitwise_and(yellow_mask, self.danger_mask)

        # 检查重叠区域是否有任何像素
        if np.any(intersection):
            object_in_proximity = True
            self.get_logger().warn('Yellow object detected in proximity zone!')
            
        # 5. 发布近距离结果
        pub_msg = Bool()
        pub_msg.data = object_in_proximity
        self.publisher_.publish(pub_msg)

        # --- 可视化 ---
        if self.enable_viz:
            # 6. 准备可视化帧
            viz_frame = cv_image.copy()
            # 绘制半透明的危险区域 (红色)
            beta = 1.0 - self.viz_alpha
            blended_frame = cv2.addWeighted(viz_frame, beta, self.viz_mask_bgr, self.viz_alpha, 0.0)
            # 查找检测到的黄色物体的轮廓
            contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # 绘制轮廓 (黄色)
            cv2.drawContours(blended_frame, contours, -1, (0, 255, 255), 2)
            # 如果在危险区，显示一个大警告
            if object_in_proximity:
                cv2.putText(blended_frame, "PROXIMITY ALERT", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)


            bev_viz_frame = cv_image.copy()
            # 绘制半透明的危险区域 (蓝色)
            bev_blended_frame = cv2.addWeighted(bev_viz_frame, beta, self.bev_viz_mask_bgr, self.viz_alpha, 0.0)
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(bev_viz_frame, red_contours, -1, (0, 0, 255), 2)
            if object_in_bev:
                cv2.putText(bev_blended_frame, "BEV RED OBJECT", (50, 100), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3, cv2.LINE_AA)
            
            # 7. 显示图像
            cv2.imshow(self.viz_window_name, blended_frame)
            cv2.imshow(self.bev_viz_window_name, bev_blended_frame)
            # 显示二值化掩码以供调试
            cv2.imshow(self.debug_window_name, yellow_mask) 
            cv2.imshow(self.bev_debug_window_name, red_mask) 
            cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = ProximityDetectorNode()
        # 检查节点是否成功初始化 (在加载掩码失败时会 shutdown)
        if rclpy.ok():
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
        # 仅在节点成功初始化并开启可视化时才关闭窗口
        if node is not None and hasattr(node, 'enable_viz') and node.enable_viz:
            cv2.destroyAllWindows()
            
        if node is not None:
            node.destroy_node()
            
        rclpy.shutdown()

if __name__ == '__main__':
    main()