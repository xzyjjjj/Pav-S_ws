from pathlib import Path
from typing import Optional, Sequence

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from sensor_msgs.msg import Image

from new.config import COSTMAP_RAW_TOPIC, OCCUPANCY_TOPIC
from new.draw_map import DrawMap


def make_costmap_qos() -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    )


class StaticCostmapPublisher(Node):
    """监听 combined 绘制出的地图图像并周期性发布 costmap。"""

    def __init__(self) -> None:
        super().__init__('final_code_costmap_pub')

        self.declare_parameter('image_path', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('layer_name', 'perception_layer')
        self.declare_parameter('resolution', 0.01)
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('mirror_y', True)
        self.declare_parameter('debug_topic', '/perception_costmap/debug_image')

        image_param = self.get_parameter('image_path').get_parameter_value().string_value
        self.image_path = Path(image_param) if image_param else None
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value or 'map'
        self.layer_name = self.get_parameter('layer_name').get_parameter_value().string_value or 'perception_layer'
        res_value = float(self.get_parameter('resolution').value)
        self.resolution = res_value if res_value > 0.0 else None
        self.publish_frequency = float(self.get_parameter('publish_frequency').value)
        self.mirror_y = bool(self.get_parameter('mirror_y').value)
        self.debug_topic = (
            self.get_parameter('debug_topic').get_parameter_value().string_value
            or '/perception_costmap/debug_image'
        )

        qos = make_costmap_qos()
        self.costmap_pub = self.create_publisher(Costmap, COSTMAP_RAW_TOPIC, qos)
        self.occupancy_pub = self.create_publisher(OccupancyGrid, OCCUPANCY_TOPIC, qos)
        self.costmap_raw_sub = self.create_subscription(
            Costmap,
            COSTMAP_RAW_TOPIC,
            self._costmap_raw_callback,
            qos,
        )

        map_origin = str(self.image_path) if self.image_path is not None else None
        self.drawer = DrawMap(map_origin)
        self.bridge = CvBridge()
        self.map_load_time = None
        self.initialized_from_raw = False
        self.received_debug_image = False

        self._load_initial_map()

        self.image_sub = self.create_subscription(
            Image,
            self.debug_topic,
            self._debug_image_callback,
            10,
        )
        self.get_logger().info(
            f"监听绘制图像: topic={self.debug_topic} raw_topic={COSTMAP_RAW_TOPIC}",
        )

        period = 1.0 / self.publish_frequency if self.publish_frequency > 0.0 else 1.0
        self.timer = self.create_timer(period, self._publish_timer)
        self.get_logger().info(
            f'静态 costmap 发布器启动: topic={OCCUPANCY_TOPIC} freq={1.0/period:.2f}Hz',
        )

    def _load_initial_map(self) -> None:
        self.get_logger().info(
            f'等待来自 {COSTMAP_RAW_TOPIC} 的初始 costmap 数据',
        )

    def _costmap_raw_callback(self, msg: Costmap) -> None:
        if msg.metadata.layer == self.layer_name:
            return
        if self.received_debug_image:
            return
        w = int(msg.metadata.size_x)
        h = int(msg.metadata.size_y)
        if w <= 0 or h <= 0:
            return
        try:
            data = np.array(msg.data, dtype=np.uint8).reshape(h, w)
        except Exception as exc:
            self.get_logger().warn(f'解析 costmap_raw 失败: {exc}')
            return

        img = np.full((h, w, 3), 255, dtype=np.uint8)
        lethal_mask = (data >= 253) & (data < 255)
        img[lethal_mask] = (0, 0, 0)

        self.drawer.map = img
        self.drawer.img_size = (w, h)

        load_time = msg.metadata.map_load_time
        if load_time and (load_time.sec != 0 or load_time.nanosec != 0):
            self.map_load_time = load_time
        else:
            self.map_load_time = msg.header.stamp

        if not self.initialized_from_raw:
            self.initialized_from_raw = True
            self.get_logger().info(
                f'已从 {COSTMAP_RAW_TOPIC} 初始化地图 ({w}x{h})',
            )

    def _debug_image_callback(self, msg: Image) -> None:
        start = time.perf_counter()
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'更新绘制地图失败: CvBridge 转换异常 {exc}')
            return

        if img is None or img.size == 0:
            self.get_logger().warn('收到空的绘制地图图像，忽略。')
            return

        self.received_debug_image = True
        self.drawer.map = img.copy()
        self.drawer.img_size = (img.shape[1], img.shape[0])
        stamp = msg.header.stamp
        if stamp is not None and (stamp.sec != 0 or stamp.nanosec != 0):
            self.map_load_time = stamp
        else:
            self.map_load_time = self.get_clock().now().to_msg()
        duration_ms = (time.perf_counter() - start) * 1000.0
        self.get_logger().debug(f'更新绘制地图耗时 {duration_ms:.1f} ms')

    def _publish_timer(self) -> None:
        start = time.perf_counter()
        outcome = 'published'
        if self.drawer.map is None:
            outcome = 'no_map'
            self.get_logger().warn('当前没有可用的地图图像，跳过发布。')
            duration_ms = (time.perf_counter() - start) * 1000.0
            self.get_logger().info(f'costmap_pub 处理耗时 {duration_ms:.1f} ms ({outcome})')
            return

        stamp = self.get_clock().now().to_msg()
        try:
            costmap_msg, occ_msg, summary = self.drawer.build_costmap_messages(
                stamp=stamp,
                map_frame=self.map_frame,
                layer_name=self.layer_name,
                resolution=self.resolution,
                mirror_y=self.mirror_y,
                map_load_time=self.map_load_time,
            )
        except Exception as exc:
            outcome = 'build_error'
            self.get_logger().error(f'生成 costmap 失败: {exc}')
            duration_ms = (time.perf_counter() - start) * 1000.0
            self.get_logger().info(f'costmap_pub 处理耗时 {duration_ms:.1f} ms ({outcome})')
            return

        self.costmap_pub.publish(costmap_msg)
        self.occupancy_pub.publish(occ_msg)

        for item in summary:
            matched = item.get('matched', 0)
            if matched > 0:
                self.get_logger().debug(
                    f"type={item['key']} occ={item.get('occ')} cost={item.get('cost')} matched={matched}",
                )

        duration_ms = (time.perf_counter() - start) * 1000.0
        self.get_logger().info(f'costmap_pub 处理耗时 {duration_ms:.1f} ms ({outcome})')

    def destroy_node(self) -> bool:
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = StaticCostmapPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
