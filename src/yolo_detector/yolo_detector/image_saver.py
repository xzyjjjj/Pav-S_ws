import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver_node')
    
        # 声明参数
        self.declare_parameter('base_save_path', '/Pav-S_ws/src/yolo_detector/runs/data_collect')
        self.declare_parameter('skip_frames', 9) # 新增：用于设置跳帧数量的参数
        
        # 获取参数
        self.base_path = self.get_parameter('base_save_path').get_parameter_value().string_value
        self.skip_frames = self.get_parameter('skip_frames').get_parameter_value().integer_value
        
        # 创建新的收集文件夹
        self.save_path = self.create_collection_folder()
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.subscription
        
        self.get_logger().info('Image saver node has been started.')
        self.get_logger().info(f'Images will be saved to: {os.path.abspath(self.save_path)}')
        self.get_logger().info(f'Saving one frame every {self.skip_frames} frames.') # 打印提示信息

        # 初始化计数器
        self.image_count = 0       # 用于文件命名 (1.png, 2.png, ...)
        self.frame_counter = 0     # 用于跳帧逻辑，记录收到的总帧数

    def create_collection_folder(self):
        """
        在基础路径下创建新的收集文件夹 (collect1, collect2, ...).
        """
        os.makedirs(self.base_path, exist_ok=True)
        i = 1
        while True:
            folder_name = f"collect{i}"
            path = os.path.join(self.base_path, folder_name)
            if not os.path.exists(path):
                os.makedirs(path)
                self.get_logger().info(f'Created directory: {path}')
                return path
            i += 1
            
    def image_callback(self, msg):
        # 每次接收到图像，总帧数计数器都+1
        self.frame_counter += 1
        
        # --- 跳帧逻辑 ---
        # 如果当前总帧数不是 skip_frames 的倍数，则直接返回，不处理这一帧
        if self.frame_counter % self.skip_frames != 0:
            return
            
        self.get_logger().info(f'Processing frame {self.frame_counter}...')
        try:
            # 将ROS的Image消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
            
        # 只有在满足跳帧条件时，用于文件命名的计数器才+1
        self.image_count += 1
        filename = f"{self.image_count}.png"
        
        # 构建完整的文件路径
        save_full_path = os.path.join(self.save_path, filename)
        
        # 保存图像
        cv2.imwrite(save_full_path, cv_image)
        self.get_logger().info(f'Saved image: {save_full_path}')

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaver()
    try:
        rclpy.spin(image_saver_node)
    except KeyboardInterrupt:
        image_saver_node.get_logger().info('Shutting down node...')
    finally:
        image_saver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()