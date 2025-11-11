import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image # 如果你想发布原始图像，但这里仅用于检测消息
from std_msgs.msg import Header
import random # 用于生成随机数据

class MockYoloPublisher(Node):
    def __init__(self):
        super().__init__('mock_yolo_publisher')
        
        # 1. 创建发布者
        self.publisher_ = self.create_publisher(
            Detection2DArray, 
            '/yolo_detections', 
            10
        )
        
        # 2. 设置定时器：每秒发布一次消息 (1.0 second interval)
        timer_period = 1.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Mock YOLO Detection Publisher Node has started.')
        self.detection_count = 0
        
    def create_mock_detection(self, label, center_x, center_y, size_x, size_y, score):
        """
        创建一个单独的 Detection2D 消息
        """
        detection = Detection2D()
        
        # A. 设置目标物体的边界框 (Bounding Box)
        bbox = BoundingBox2D()
        bbox.center.position.x = float(center_x)
        bbox.center.position.y = float(center_y)
        bbox.size_x = float(size_x)
        bbox.size_y = float(size_y)
        detection.bbox = bbox
        
        # B. 设置目标物体的类别和置信度 (Results)
        # vision_msgs 使用 ObjectHypothesisWithPose 来表示检测结果
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label             # 例如 'person', 'car', 'chair'
        hypothesis.hypothesis.score = float(score)   # 置信度 (0.0 to 1.0)
        detection.results.append(hypothesis)

        return detection

    def timer_callback(self):
        """
        定时器回调函数，用于构建并发布 Detection2DArray
        """
        self.detection_count += 1
        
        # 1. 创建 Detection2DArray 消息容器
        detections_array_msg = Detection2DArray()
        
        # 2. 填充 Header（非常重要，用于指定时间戳和坐标系）
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link' # 假设检测是相对于 'camera_link' 坐标系
        detections_array_msg.header = header

        # 3. 创建并添加假检测结果 (Mock Detections)

        person_detection = self.create_mock_detection(
            label='num_5', 
            center_x=150, center_y=300, 
            size_x=100, size_y=250, 
            score=0.95
        )
        detections_array_msg.detections.append(person_detection)
        
        # # 检测 2: 一辆车在图像右侧（带随机置信度）
        # car_score = random.uniform(0.7, 0.9) # 随机生成置信度
        # car_detection = self.create_mock_detection(
        #     label='car', 
        #     center_x=500, center_y=450, 
        #     size_x=200, size_y=150, 
        #     score=car_score
        # )
        # detections_array_msg.detections.append(car_detection)
        
        # 4. 发布消息
        self.publisher_.publish(detections_array_msg)
        
        self.get_logger().info(
            f'[{self.detection_count}] Publishing Detection2DArray with {len(detections_array_msg.detections)} detections.'
        )

def main(args=None):
    rclpy.init(args=args)
    mock_publisher = MockYoloPublisher()
    
    try:
        rclpy.spin(mock_publisher)
    except KeyboardInterrupt:
        pass
        
    # 销毁节点并关闭 rclpy
    mock_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()