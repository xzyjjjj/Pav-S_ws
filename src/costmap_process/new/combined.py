from typing import Dict, List, Optional, Sequence, Tuple

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from .config import COSTMAP_RAW_TOPIC, LOCAL, MAP_ORIGIN_PATH, OCCUPANCY_COLOR_MAP, OCCUPANCY_TOPIC
from .data_interface import (
    get_all_type,
    get_specific_bbox,
    parse_bbox_from_obj,
    tf_matrix_from_tr_and_quat,
)
from .draw_map import DrawMap
from .filter_point import FilterPoints
from .tf_transformer import TfTransformer

SHOW_TIME = False  # 设为 True 时输出各步骤耗时

Pixel = Tuple[int, int]


def make_costmap_qos() -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    )


class PerceptionCostmapNode(Node):
    """读取感知输出并生成静态 costmap。"""

    def __init__(self) -> None:
        super().__init__('perception_costmap_node')
        self.bridge = CvBridge()
        self.drawer = DrawMap(MAP_ORIGIN_PATH)
        self.filter = FilterPoints()
        self.transformer = TfTransformer()
        self._tf_fallback_warned = False

        qos = make_costmap_qos()
        self.costmap_pub = self.create_publisher(Costmap, COSTMAP_RAW_TOPIC, qos)
        self.occupancy_pub = self.create_publisher(OccupancyGrid, OCCUPANCY_TOPIC, qos)
        self.debug_img_pub = self.create_publisher(Image, '/perception_costmap/debug_image', 1)

        self.tf_buffer = None
        self.tf_listener = None
        try:
            from tf2_ros import Buffer, TransformListener

            self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'TF 初始化失败: {exc}')

        if LOCAL:
            self._init_local_mode()
        else:
            self._init_ros_mode()

        self.map_load_time = self.get_clock().now().to_msg()
        self.frame_index = 0

    def _init_ros_mode(self) -> None:
        self.get_logger().info('运行在 ROS 模式，订阅 rgb_img 与 yolo_detections')

        self.image_sub = self.create_subscription(Image, 'rgb_img', self._rgb_callback, 10)
        self.red_image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.det_sub = self.create_subscription(Detection2DArray, 'yolo_detections', self._dets_callback, 10)

        self._latest_image: Optional[Image] = None
        self._latest_dets: Optional[Detection2DArray] = None

    def _init_local_mode(self) -> None:
        self.get_logger().info('运行在 LOCAL 模式（暂未实现）')

    def _log_step_time(self, label: str, start: float, last: float) -> float:
        now = time.perf_counter()
        if SHOW_TIME:
            self.get_logger().info(
                f'{label}耗时 {(now - last) * 1000.0:.1f} ms (累计 {(now - start) * 1000.0:.1f} ms)',
            )
        return now

    def image_callback(self, img_msg: Image) -> None:
        """独立处理 rgb_img，基于颜色区分绘制红色占据区域。"""
        log = self.get_logger()
        start = time.perf_counter()
        outcome = 'success'
        try:
            stamp = img_msg.header.stamp
            log.info(f"图像消息时间戳: sec={getattr(stamp, 'sec', None)}, nanosec={getattr(stamp, 'nanosec', None)}")
            try:
                img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            except Exception as exc:
                outcome = 'cv_bridge_error'
                log.error(f'图像转换失败(红区): {exc}')
                return

            if img is None or img.size == 0:
                outcome = 'empty_image'
                log.warn('收到的红区图像为空或无效')
                return

            tf_matrix = self._lookup_tf('map', 'body', img_msg.header.stamp)
            if tf_matrix is None:
                outcome = 'missing_tf'
                log.warn('TF map->body 查找失败（红区），跳过该帧')
                return

            try:
                from .color_segment import segment_red_road_sign
                mask = segment_red_road_sign(img)
            except Exception as exc:
                outcome = 'segmentation_error'
                log.warn(f'红色区域分割失败: {exc}')
                return

            if mask is None:
                outcome = 'empty_mask'
                log.info('红色区域分割返回空掩膜')
                return

            ys, xs = np.where(mask > 0)
            if len(xs) == 0:
                outcome = 'no_pixels'
                log.info('红色区域分割未找到有效像素')
                return

            projected: List[Pixel] = []
            for u, v in zip(xs, ys):
                try:
                    res = self.transformer.calc_point_on_origin_map((int(u), int(v)), tf_matrix, 0)
                except Exception:
                    continue
                map_px = res.get('map_pixel') if res else None
                if map_px is None or len(map_px) < 2:
                    continue
                projected.append((int(map_px[0]), int(map_px[1])))

            if not projected:
                outcome = 'no_projected_pixels'
                log.info('红色区域投影后无有效像素')
                return

            unique_pixels = list(dict.fromkeys(projected))
            log.info(f"红色区域投影像素 {len(unique_pixels)} 个")
            self._draw_and_publish({'red_zone': unique_pixels})
            outcome = f'success/{len(unique_pixels)}px'
        finally:
            duration_ms = (time.perf_counter() - start) * 1000.0
            log.info(f'image_callback 耗时 {duration_ms:.1f} ms ({outcome})')

    def _rgb_callback(self, msg: Image) -> None:
        self._latest_image = msg
        self._process_if_ready()

    def _dets_callback(self, msg: Detection2DArray) -> None:
        self._latest_dets = msg
        self._process_if_ready()

    def _process_if_ready(self) -> None:
        if self._latest_image is None or self._latest_dets is None:
            return
        img_msg = self._latest_image
        dets_msg = self._latest_dets
        self._latest_image = None
        self._latest_dets = None
        self._handle_frame(img_msg, dets_msg)

    def _handle_frame(self, img_msg: Image, dets_msg: Detection2DArray) -> None:
        self.frame_index += 1
        log = self.get_logger()
        log.info(f'处理帧 {self.frame_index}, stamp={img_msg.header.stamp.sec}.{img_msg.header.stamp.nanosec:09d}')
        step_start = time.perf_counter()
        last_step = step_start

        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as exc:
            log.error(f'图像转换失败: {exc}')
            return
        last_step = self._log_step_time('图像转换', step_start, last_step)

        det_list = parse_bbox_from_obj(dets_msg)
        last_step = self._log_step_time('解析检测结果', step_start, last_step)
        if not det_list:
            log.info('检测结果为空，跳过该帧')
            return

        tf_matrix = self._lookup_tf('map', 'body', img_msg.header.stamp)
        last_step = self._log_step_time('查询 TF', step_start, last_step)
        if tf_matrix is None:
            log.warn('TF map->body 查找失败，跳过该帧')
            return

        pixels_by_type = self._project_detections(img, det_list, tf_matrix)
        last_step = self._log_step_time('投影检测', step_start, last_step)
        if not pixels_by_type:
            log.info('投影后无有效像素')
            return

        total_pixels = sum(len(pixels) for pixels in pixels_by_type.values())
        log.info(f'投影成功: {len(pixels_by_type)} 类别, {total_pixels} 个像素')
        self._draw_and_publish(pixels_by_type)
        last_step = self._log_step_time('绘制发布', step_start, last_step)
        if SHOW_TIME:
            total_ms = (time.perf_counter() - step_start) * 1000.0
            log.info(f'帧处理总耗时 {total_ms:.1f} ms')

    def _lookup_tf(self, target: str, source: str, stamp: Time) -> Optional[np.ndarray]:
        if self.tf_buffer is None:
            return None

        def _transform_to_matrix(transform) -> Optional[np.ndarray]:
            if transform is None:
                return None
            tr = transform.transform.translation
            rot = transform.transform.rotation
            return tf_matrix_from_tr_and_quat(
                [tr.x, tr.y, tr.z],
                [rot.x, rot.y, rot.z, rot.w],
            )

        try:
            ready = self.tf_buffer.can_transform(target, source, stamp, timeout=Duration(seconds=0.3))
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'TF can_transform 异常: {exc}')
            ready = False

        if ready:
            try:
                transform = self.tf_buffer.lookup_transform(target, source, stamp, timeout=Duration(seconds=0.0))
                return _transform_to_matrix(transform)
            except Exception as exc:  # pragma: no cover
                self.get_logger().warn(f'TF 查找异常: {exc}')

        # 回退：尝试使用最新可用的 TF
        try:
            latest = self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.3))
            if not self._tf_fallback_warned:
                self.get_logger().warn('TF 使用最新可用变换进行回退，可能存在 车辆位姿延迟')
                self._tf_fallback_warned = True
            return _transform_to_matrix(latest)
        except Exception:
            return None

    def _project_detections(
        self,
        img: np.ndarray,
        det_list: Sequence[Dict],
        tf_matrix: np.ndarray,
    ) -> Dict[str, List[Pixel]]:
        result: Dict[str, List[Pixel]] = {}

        for det_type in get_all_type(det_list):
            bbox_pts = get_specific_bbox(det_type, det_list)
            bbox_int = None
            if bbox_pts:
                bbox_int = [[int(round(x)), int(round(y))] for x, y in bbox_pts]

            masked_img, _ = self.filter.extract_points(img, bbox_int, det_type)
            mask = np.any(masked_img != 0, axis=2)
            ys, xs = np.where(mask)
            if len(xs) == 0:
                continue

            pixels: List[Pixel] = []
            for u, v in zip(xs, ys):
                try:
                    res = self.transformer.calc_point_on_origin_map((u, v), tf_matrix, 0)
                except Exception as exc:
                    self.get_logger().warn(f'像素 ({u},{v}) 投影失败: {exc}')
                    continue
                map_px = res.get('map_pixel')
                if map_px is None:
                    continue
                pixels.append((int(map_px[0]), int(map_px[1])))

            if pixels:
                result.setdefault(det_type, []).extend(pixels)
                self.get_logger().info(f'类别 {det_type} 投影像素 {len(pixels)} 个')

        return result

    def _draw_and_publish(self, pixels_by_type: Dict[str, List[Pixel]]) -> None:
        any_drawn = False
        for det_type, pixels in pixels_by_type.items():
            if not pixels:
                continue
            color_info = OCCUPANCY_COLOR_MAP.get(det_type) or OCCUPANCY_COLOR_MAP.get(str(det_type))
            rgb = tuple(int(c) for c in color_info['rgb']) if color_info and 'rgb' in color_info else (0, 0, 255)
            try:
                self.drawer.draw_point_on_map(pixels, color=rgb, radius=2)
                any_drawn = True
            except Exception as exc:
                self.get_logger().warn(f'绘制 {det_type} 失败: {exc}')

        if not any_drawn:
            self.get_logger().info('没有新像素，跳过发布')
            return

        stamp = self.get_clock().now().to_msg()
        costmap_msg, occ_msg, summary = self.drawer.build_costmap_messages(
            stamp=stamp,
            map_frame='map',
            layer_name='perception_layer',
            mirror_y=True,
            map_load_time=self.map_load_time,
        )

        self.costmap_pub.publish(costmap_msg)
        self.occupancy_pub.publish(occ_msg)

        for item in summary:
            if item.get('matched', 0) > 0:
                self.get_logger().info(
                    f"type={item['key']} occ={item.get('occ')} cost={item.get('cost')} matched={item['matched']}",
                )

        try:
            debug_img = self.drawer.map
            msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            msg.header.stamp = stamp
            msg.header.frame_id = 'map'
            self.debug_img_pub.publish(msg)
            self.get_logger().info("发布成功")
        except Exception as exc:
            self.get_logger().warn(f'发布调试图失败: {exc}')


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionCostmapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
