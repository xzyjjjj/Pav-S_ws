#!/usr/bin/env python3
import os
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import cv2
import json
from datetime import datetime

class RGBDCapture(Node):
    def __init__(self, color_topic, depth_topic, out_dir, timeout):
        super().__init__('rgbd_capture')
        self.bridge = CvBridge()
        self.out_dir = out_dir
        self.timeout = timeout
        os.makedirs(self.out_dir, exist_ok=True)

        # image subs with sensor QoS
        self.color_sub = Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.cb)

        # optional camera info (best-effort; not synchronized)
        self.color_info_msg = None
        self.depth_info_msg = None
        self.create_subscription(CameraInfo, color_topic.replace('image_raw', 'camera_info'),
                                 self._color_info_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, depth_topic.replace('image_raw', 'camera_info'),
                                 self._depth_info_cb, qos_profile_sensor_data)

        self.captured = False
        self.start_time = self.get_clock().now()

        self.get_logger().info(f'等待图像: color={color_topic}, depth={depth_topic}')
        self.get_logger().info(f'保存目录: {self.out_dir}, 超时: {self.timeout}s')

        self.timer = self.create_timer(0.2, self._check_timeout)

    def _color_info_cb(self, msg: CameraInfo):
        self.color_info_msg = msg

    def _depth_info_cb(self, msg: CameraInfo):
        self.depth_info_msg = msg

    def _check_timeout(self):
        if self.captured:
            return
        if (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 > self.timeout:
            self.get_logger().error('等待图像超时，未能捕获到同步的 RGBD 帧')
            rclpy.shutdown()

    def cb(self, color_msg: Image, depth_msg: Image):
        if self.captured:
            return
        stamp = color_msg.header.stamp
        ts = f'{stamp.sec}.{stamp.nanosec:09d}'
        ts_human = datetime.utcnow().strftime('%Y%m%d_%H%M%S_%f')[:-3]

        # color
        try:
            # 优先保留原始编码；若为 rgb8，保存为 BGR PNG
            encoding = color_msg.encoding.lower()
            img_c = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8' if encoding in ('rgb8', 'bgr8') else None)
        except Exception as e:
            self.get_logger().error(f'彩色图转换失败: {e}')
            return

        # depth
        try:
            depth_enc = depth_msg.encoding.upper()
            depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=depth_msg.encoding)
            if depth_enc == '16UC1':
                depth_u16 = depth_cv.astype(np.uint16)
            elif depth_enc == '32FC1':
                # meters -> mm 16-bit
                depth_m = np.nan_to_num(depth_cv, nan=0.0, posinf=0.0, neginf=0.0)
                depth_u16 = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
            else:
                self.get_logger().warn(f'未知深度编码 {depth_msg.encoding}，尝试以 16UC1 方式保存')
                depth_u16 = depth_cv.astype(np.uint16)
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {e}')
            return

        # write files
        color_path = os.path.join(self.out_dir, f'color_{ts_human}.png')
        depth_path = os.path.join(self.out_dir, f'depth_{ts_human}.png')
        meta_path  = os.path.join(self.out_dir, f'meta_{ts_human}.json')

        ok_c = cv2.imwrite(color_path, img_c)
        ok_d = cv2.imwrite(depth_path, depth_u16)
        if not ok_c or not ok_d:
            self.get_logger().error('写入 PNG 失败')
            return

        meta = {
            'ros_stamp': ts,
            'color_topic': self.color_sub.topic,
            'depth_topic': self.depth_sub.topic,
            'depth_encoding': depth_msg.encoding,
            'color_encoding': color_msg.encoding,
            'color_frame': color_msg.header.frame_id,
            'depth_frame': depth_msg.header.frame_id,
            'files': {'color': color_path, 'depth': depth_path}
        }
        # dump intrinsics if available
        if self.color_info_msg:
            meta['color_camera_info'] = {
                'k': list(self.color_info_msg.k),
                'd': list(self.color_info_msg.d),
                'r': list(self.color_info_msg.r),
                'p': list(self.color_info_msg.p),
                'distortion_model': self.color_info_msg.distortion_model,
                'width': self.color_info_msg.width, 'height': self.color_info_msg.height
            }
        if self.depth_info_msg:
            meta['depth_camera_info'] = {
                'k': list(self.depth_info_msg.k),
                'd': list(self.depth_info_msg.d),
                'r': list(self.depth_info_msg.r),
                'p': list(self.depth_info_msg.p),
                'distortion_model': self.depth_info_msg.distortion_model,
                'width': self.depth_info_msg.width, 'height': self.depth_info_msg.height
            }
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        self.get_logger().info(f'已保存: {color_path} 与 {depth_path}')
        self.captured = True
        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description='Capture one RGB-D pair and save as PNG.')
    parser.add_argument('--color-topic', default='/camera/color/image_raw')
    parser.add_argument('--depth-topic', default='/camera/depth/image_raw')
    parser.add_argument('--out-dir', default='rgbd_out')
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = RGBDCapture(args.color_topic, args.depth_topic, args.out_dir, args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()