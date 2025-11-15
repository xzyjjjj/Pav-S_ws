#!/usr/bin/env python3
"""
订阅 YOLO Detection2DArray，提取检测框中心点并转换为地图像素坐标。
输出内容：[(type, map_point_x, map_point_y), ...]
"""

from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from vision_msgs.msg import Detection2DArray

from .data_interface import tf_matrix_from_tr_and_quat
from .tf_transformer import TfTransformer


class YoloCenterProjector(Node):
    def __init__(self) -> None:
        super().__init__('yolo_center_projector')

        self.declare_parameter('detection_topic', '/yolo_detections')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'body')

        self.detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value

        self.transformer = TfTransformer()
        self.tf_buffer = None
        self.tf_listener = None

        try:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'TF 初始化失败: {exc}')

        self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.detection_callback,
            10,
        )
        self.get_logger().info(f'已订阅 {self.detection_topic}')

    def detection_callback(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            return

        tf_matrix = self._lookup_tf(self.target_frame, self.source_frame, msg.header.stamp)
        if tf_matrix is None:
            self.get_logger().warn('未获取到 TF，跳过该帧')
            return

        results: List[Tuple[str, Tuple[int, int]]] = []
        for det in msg.detections:
            if not det.results:
                continue

            det_type = det.results[0].hypothesis.class_id or str(det.results[0].hypothesis.id)

            center = det.bbox.center
            cx = getattr(center, 'x', None)
            cy = getattr(center, 'y', None)
            if (cx is None or cy is None) and hasattr(center, 'position'):
                position = center.position
                cx = getattr(position, 'x', None)
                cy = getattr(position, 'y', None)
            if cx is None or cy is None:
                self.get_logger().warn('检测框中心缺少 x/y，跳过该目标')
                continue

            cx = int(round(cx))
            cy = int(round(cy))

            try:
                res = self.transformer.calc_point_on_origin_map((cx, cy), tf_matrix, 0)
            except Exception as exc:
                self.get_logger().warn(f'像素 ({cx},{cy}) 投影失败: {exc}')
                continue

            map_px = res.get('map_pixel') if res else None
            if map_px is None or len(map_px) < 2:
                continue

            map_point = (int(map_px[0]), int(map_px[1]))
            results.append((det_type, map_point))

        if results:
            self.get_logger().info(f'当前帧 {len(results)} 个目标:')
            for det_type, (mx, my) in results:
                self.get_logger().info(f'  type={det_type}, map_point=({mx}, {my})')

    def _lookup_tf(self, target: str, source: str, stamp: Time) -> Optional[Sequence[Sequence[float]]]:
        if self.tf_buffer is None:
            return None

        def to_matrix(transform) -> Optional[Sequence[Sequence[float]]]:
            if transform is None:
                return None
            tr = transform.transform.translation
            rot = transform.transform.rotation
            return tf_matrix_from_tr_and_quat(
                [tr.x, tr.y, tr.z],
                [rot.x, rot.y, rot.z, rot.w],
            )

        try:
            if self.tf_buffer.can_transform(target, source, stamp, timeout=Duration(seconds=0.3)):
                transform = self.tf_buffer.lookup_transform(target, source, stamp, timeout=Duration(seconds=0.0))
                return to_matrix(transform)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'TF 查找异常: {exc}')

        try:
            latest = self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.3))
            self.get_logger().warn('使用最新 TF 回退，可能存在时间误差')
            return to_matrix(latest)
        except Exception:
            return None


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = YoloCenterProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()