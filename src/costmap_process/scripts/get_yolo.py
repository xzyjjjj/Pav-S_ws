import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import cv2
from cv_bridge import CvBridge
import os
import json
from datetime import datetime

OUTPUT_DIR = '/Pav-S_ws/src/costmap_process/detection_output_color'

class DetectionSaver(Node):
    def __init__(self):
        super().__init__('detection_saver')
        self.bridge = CvBridge()
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self.create_subscription(Detection2DArray, '/yolo_detections', self.detection_callback, 10)
        self.create_subscription(Image, '/rgb_img', self.image_callback, 10)
        self.create_subscription(Image, '/depth_img', self.depth_callback, 10)

    def detection_callback(self, msg):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        det_list = []
        for det in msg.detections:
            bbox = det.bbox
            if det.results:
                class_id = det.results[0].hypothesis.class_id
                score = det.results[0].hypothesis.score
            else:
                class_id = None
                score = None
            det_list.append({
                'center_x': bbox.center.position.x,
                'center_y': bbox.center.position.y,
                'size_x': bbox.size_x,
                'size_y': bbox.size_y,
                'class_id': class_id,
                'score': score
            })
        json_path = os.path.join(OUTPUT_DIR, f'detection_{ts}.json')
        with open(json_path, 'w') as f:
            json.dump(det_list, f, indent=2)
        self.get_logger().info(f'检测框已保存: {json_path}')

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            img_path = os.path.join(OUTPUT_DIR, f'rgb_{ts}.png')
            cv2.imwrite(img_path, cv_img)
            self.get_logger().info(f'RGB图片已保存: {img_path}')
        except Exception as e:
            self.get_logger().error(f'RGB图片保存失败: {e}')

    def depth_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, '16UC1')  # 深度图像通常为16位
            ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            img_path = os.path.join(OUTPUT_DIR, f'depth_{ts}.png')
            cv2.imwrite(img_path, cv_img)
            self.get_logger().info(f'深度图片已保存: {img_path}')
        except Exception as e:
            self.get_logger().error(f'深度图片保存失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DetectionSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()