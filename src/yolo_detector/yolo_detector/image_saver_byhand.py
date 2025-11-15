import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from rclpy.qos import QoSProfile, qos_profile_sensor_data

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver_node')
    
        # 声明保存路径参数
        self.declare_parameter('base_save_path', '/Pav-S_ws/src/yolo_detector/runs/bev/data_collect')
        
        # 获取参数
        self.base_path = self.get_parameter('base_save_path').get_parameter_value().string_value
        
        # 创建新的收集文件夹
        self.save_path = self.create_collection_folder()
        
        self.bridge = CvBridge()
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            # '/camera/color/image_raw',
            '/bev/debug_image',
            self.image_callback,
            10)
        self.subscription
        
        # 初始化用于文件命名的计数器
        self.image_count = 0
        
        self.get_logger().info('Image saver node has been started.')
        self.get_logger().info(f'Images will be saved to: {os.path.abspath(self.save_path)}')
        self.get_logger().info("--- Press 's' in the display window to save an image. ---")
        self.get_logger().info("--- Press 'q' in the display window to quit. ---")


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
        try:
            # 将ROS的Image消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
            
        # 显示图像
        cv2.imshow("Camera Feed | Press 's' to save, 'q' to quit", cv_image)
        key = cv2.waitKey(1) & 0xFF

        # 如果按下 's' 键，则保存图片
        if key == ord('s'):
            self.image_count += 1
            filename = f"{self.image_count}.png"
            save_full_path = os.path.join(self.save_path, filename)
            
            # 保存图像
            cv2.imwrite(save_full_path, cv_image)
            self.get_logger().info(f'Saved image: {save_full_path}')

        # 如果按下 'q' 键，则关闭程序
        elif key == ord('q'):
            self.get_logger().info('"q" pressed, shutting down node.')
            # 在销毁节点前关闭所有OpenCV窗口
            cv2.destroyAllWindows()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaver()
    try:
        # 当节点被销毁时 (例如在callback中调用self.destroy_node()), spin会退出
        rclpy.spin(image_saver_node)
    except KeyboardInterrupt:
        image_saver_node.get_logger().info('Shutting down node via KeyboardInterrupt...')
    finally:
        # 确保即使节点已经被销毁，也能安全地尝试销毁
        if rclpy.ok() and image_saver_node.handle:
            image_saver_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows() # 确保任何情况下窗口都被关闭


if __name__ == '__main__':
    main()