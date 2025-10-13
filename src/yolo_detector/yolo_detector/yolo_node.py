#!/usr/bin/env python3
# coding=utf-8
from yolo_detector.detector_module import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from std_msgs.msg import Header


#* 封装ros交互，订阅图像、发布检测结果
class RosYOLODetector(Node):
    def __init__(self):
        super().__init__('ros_yolo_detector')  # 调用父类构造器，传入节点名

        # 1. 声明参数
        self.declare_parameter('weights_path', 'yolov5l.pt')
        self.declare_parameter('config_path', './data/coco.yaml')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('target_lab_name', '') # ? 设置一个目标检测类别。用于检测对单个类别物体的识别效果。
        self.declare_parameter('enable_vis', False)
        self.declare_parameter('skip_frames', 3)

        # 2. 获取参数值
        weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        target_lab_name = self.get_parameter('target_lab_name').get_parameter_value().string_value
        enable_vis = self.get_parameter('enable_vis').get_parameter_value().bool_value
        skip_frames = self.get_parameter('skip_frames').get_parameter_value().integer_value

        # 3. 使用获取到的变量来初始化模型对象
        self.model = DetectorYolov5(cfgfile=config_path, weightfile=weights_path)
        if self.model.device == "cuda": 
            # 打印所使用的 GPU 信息
            gpu_name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "Unknown GPU"
            self.get_logger().info(f"✅ Detector initialized using: CUDA ({gpu_name})")

        else:
            self.get_logger().warn("❌ Detector initialized using: CPU (CUDA not available)")
            self.get_logger().info("Using CPU instead for detection.") # 将提示信息改为 info 级别
            
        
        # ? （测试用）设置一个目标检测类别。用于检测对单个类别物体的识别效果。
        self.target_lab_name = target_lab_name

        self.bridge = CvBridge()

        # 订阅图像话题
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        
        # 订阅深度图像话题
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)

        # 发布检测框（只保留 vehicle）
        self.detection_pub = self.create_publisher(Detection2DArray, "/yolo_detections", 10)
        
        # 发布RGB图像
        self.rgb_pub = self.create_publisher(Image, "/rgb_img", 10)
        
        # 发布深度图像
        self.depth_pub = self.create_publisher(Image, "/depth_img", 10)

        self.enable_vis = enable_vis

        # 控制帧率
        self.frame_count = 0  # 初始化接收的图片总数
        self.skip_frames = skip_frames  # 每第3帧处理一次
        
        # 存储最新的深度图像
        self.latest_depth_image = None
        self.latest_depth_header = None

    def depth_callback(self, depth_msg):
        """深度图像回调函数，存储最新的深度图像"""
        self.latest_depth_image = depth_msg

    def image_callback(self, ros_img):
        self.frame_count += 1
        # 跳帧处理
        if self.frame_count % (self.skip_frames + 1) != 0:
            return

        try: # 将 ROS 图像消息转换为 OpenCV 的 BGR8 格式图像
            cv_image = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return
        # 将 BGR 图像转为 RGB 图像，因为 YOLOv5 和 PIL 库通常使用 RGB 格式
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_pil = PILImage.fromarray(image) # 将 NumPy 数组格式的图像转换为 PIL 图像对象，以供 DetectorYolov5 类使用。
        
        #* 调用模型进行检测
        result_image, bbox = self.model.detect(image_pil)
        bbox = bbox.cpu().detach().numpy()

        # 发布 RGB 图像
        self.rgb_pub.publish(ros_img)
        
        # 发布深度图像（如果存在）
        if self.latest_depth_image is not None:
            self.depth_pub.publish(self.latest_depth_image)

        # 发布 Detection2DArray 消息
        msg_array = Detection2DArray()
        msg_array.header = ros_img.header # 将输入图像的 header（包含时间戳和坐标系信息）赋给输出消息

        # # 没有检测到目标时，填充一个无效的目标信息到数组中
        # if len(bbox) == 0:
        #     detection = Detection2D()
        #     detection.bbox.center.position.x = 99999.0 # TODO：表示未检测到目标
        #     detection.bbox.center.position.y = 99999.0
        #     detection.bbox.size_x = 99999.0
        #     detection.bbox.size_y = 99999.0

        #     hypothesis = ObjectHypothesisWithPose()
        #     hypothesis.hypothesis.class_id = "0"  # vehicle 类别ID
        #     hypothesis.hypothesis.score = 1.0
        #     detection.results.append(hypothesis)
        #     msg_array.detections.append(detection)

        top_label = bbox[:, 5]  # 从 bbox 数组中提取所有检测框的类别 ID
        top_conf = bbox[:, 4]   # 提取置信度信息
        top_boxes = bbox[:, :4] # 提取 bbox 位置信息

        if(self.target_lab_name != ''):
            if self.target_lab_name not in self.model.yaml["names"].values():
                self.get_logger().warn(f"{self.target_lab_name} 类别不在 class_names 中！")
                return
            # 将目标名称转换为类别 ID。
            target_class_id = self.model.name2id(self.target_lab_name)
            # 创建一个布尔掩码，用于筛选出所有类别为 self.target_lab_name 的检测结果。
            target_class_mask = (top_label == target_class_id)

            if not np.any(target_class_mask):
                self.get_logger().info(f"No {self.target_lab_name} detected, skipping publish.")

            top_label = top_label[target_class_mask]
            top_conf = top_conf[target_class_mask]
            top_boxes = top_boxes[target_class_mask]

        # 填充并发布消息
        for i in range(len(top_boxes)):
            box = top_boxes[i]
            conf = top_conf[i]      
            cls_id = top_label[i]   

            detection = Detection2D()
            
            # 填充bbox
            detection.bbox.center.position.x = float(box[0])
            detection.bbox.center.position.y = float(box[1])
            detection.bbox.size_x = float(box[2])
            detection.bbox.size_y = float(box[3])
            
            # 填充类别和置信度
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self.model.id2name(cls_id)
            hypothesis.hypothesis.score = float(conf)
            detection.results.append(hypothesis)

            msg_array.detections.append(detection)

        self.detection_pub.publish(msg_array)

        # 可视化检测结果（如果启用）
        result_image = cv2.cvtColor(np.asarray(result_image), cv2.COLOR_RGB2BGR)
        if self.enable_vis:
            cv2.imshow("YOLO Detection", result_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    detector = RosYOLODetector()
    detector.get_logger().info("ROS YOLO detector with Detection2DArray output started.")
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()