import math
import os
import threading
from typing import List, Optional

import numpy as np
from PIL import Image, ImageOps

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Pose, Quaternion
from nav2_msgs.msg import Costmap, CostmapMetaData
from nav_msgs.msg import OccupancyGrid, MapMetaData as NavMapMetaData


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


class PngCostmapPublisher(Node):
    def __init__(self) -> None:
        super().__init__('png_costmap_publisher')

        self.declare_parameter('image_path', '')
        self.declare_parameter('costmap_topic', 'processed_costmap')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('layer_name', 'png_layer')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('origin', [0.0, 0.0, 0.0])
        self.declare_parameter('publish_frequency', 5.0)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('free_threshold', 200)
        self.declare_parameter('occupied_value', 254)
        self.declare_parameter('free_value', 0)
        self.declare_parameter('unknown_value', 255)
        self.declare_parameter('negate', False)
        self.declare_parameter('watch_file', True)
        self.declare_parameter('alpha_unknown_threshold', 1)
        self.declare_parameter('qos_reliability', 'reliable')

        self._lock = threading.Lock()

        self.image_path = self.get_parameter('image_path').get_parameter_value().string_value
        if not self.image_path:
            self.get_logger().error('Parameter "image_path" must be provided.')
            raise RuntimeError('Missing required parameter: image_path')

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.layer_name = self.get_parameter('layer_name').get_parameter_value().string_value
        self.resolution = float(self.get_parameter('resolution').value)
        self.publish_frequency = float(self.get_parameter('publish_frequency').value)
        self.negate = bool(self.get_parameter('negate').value)
        self.watch_file = bool(self.get_parameter('watch_file').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.occupied_value = int(self.get_parameter('occupied_value').value)
        self.free_value = int(self.get_parameter('free_value').value)
        self.unknown_value = int(self.get_parameter('unknown_value').value)
        self.alpha_unknown_threshold = int(self.get_parameter('alpha_unknown_threshold').value)

        origin_param = self.get_parameter('origin').value
        if not isinstance(origin_param, (list, tuple)) or len(origin_param) != 3:
            self.get_logger().warn('Origin parameter invalid; expected [x, y, yaw]. Using defaults (0,0,0).')
            origin_param = [0.0, 0.0, 0.0]
        self.origin = tuple(float(v) for v in origin_param)

        qos_reliability = self.get_parameter('qos_reliability').value
        reliability_policy = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        if qos_reliability == 'best_effort':
            reliability_policy = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
            reliability=reliability_policy,
        )

        topic_name = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(Costmap, topic_name, qos_profile)
        # also publish a nav_msgs/OccupancyGrid so RViz can visualize it directly
        occ_topic = topic_name + '_occupancy'
        self.occupancy_publisher_ = self.create_publisher(OccupancyGrid, occ_topic, qos_profile)

        self.map_load_time = self.get_clock().now().to_msg()
        self.metadata_template: Optional[CostmapMetaData] = None
        self.costmap_data: Optional[List[int]] = None
        self.last_mtime: Optional[float] = None

        self._reload_costmap(initial=True)

        period = 1.0 / self.publish_frequency if self.publish_frequency > 0 else 1.0
        self.timer = self.create_timer(period, self._publish_costmap)
        self.get_logger().info(
            f'Publishing PNG-based costmap on topic "{topic_name}" at {self.publish_frequency:.2f} Hz')

    def _reload_costmap(self, initial: bool = False) -> None:
        try:
            mtime = os.path.getmtime(self.image_path)
        except OSError as exc:
            self.get_logger().error(f'Failed to access image "{self.image_path}": {exc}')
            raise

        if not initial and self.last_mtime is not None and mtime <= self.last_mtime:
            return

        self.get_logger().info(f'Loading PNG costmap from {self.image_path}')

        with Image.open(self.image_path) as image:
            alpha_channel = None
            if 'A' in image.getbands():
                alpha_channel = np.array(image.split()[-1], dtype=np.uint8)
            image = image.convert('L')
            if self.negate:
                image = ImageOps.invert(image)
            gray = np.array(image, dtype=np.uint8)

        if gray.ndim != 2:
            raise RuntimeError('Expected a 2-D grayscale image.')

        height, width = gray.shape
        costmap_array = np.full((height, width), self.unknown_value, dtype=np.uint8)

        if alpha_channel is not None:
            transparent_mask = alpha_channel <= self.alpha_unknown_threshold
            costmap_array[transparent_mask] = self.unknown_value
        else:
            transparent_mask = np.zeros_like(gray, dtype=bool)

        occupied_mask = gray <= self.occupied_threshold
        free_mask = (gray >= self.free_threshold) & (~transparent_mask)
        unknown_mask = ~(occupied_mask | free_mask | transparent_mask)

        costmap_array[occupied_mask] = self.occupied_value
        costmap_array[free_mask] = self.free_value
        costmap_array[unknown_mask] = self.unknown_value

        costmap_array = np.flipud(costmap_array)

        metadata = CostmapMetaData()
        metadata.resolution = float(self.resolution)
        metadata.size_x = int(width)
        metadata.size_y = int(height)
        metadata.layer = self.layer_name
        load_time = self.get_clock().now().to_msg()
        metadata.map_load_time = load_time
        metadata.update_time = load_time

        origin_pose = Pose()
        origin_pose.position.x = float(self.origin[0])
        origin_pose.position.y = float(self.origin[1])
        origin_pose.position.z = 0.0
        origin_pose.orientation = yaw_to_quaternion(float(self.origin[2]))
        metadata.origin = origin_pose

        with self._lock:
            self.metadata_template = metadata
            self.costmap_data = costmap_array.flatten().tolist()
            self.last_mtime = mtime
            self.map_load_time = load_time
            self.get_logger().info(f'PNG costmap loaded ({width}x{height})')

    def _publish_costmap(self) -> None:
        if self.watch_file:
            try:
                current_mtime = os.path.getmtime(self.image_path)
            except OSError as exc:
                self.get_logger().error(f'File access error for "{self.image_path}": {exc}')
                return

            with self._lock:
                last_mtime = self.last_mtime

            if last_mtime is None or current_mtime > last_mtime:
                try:
                    self._reload_costmap()
                except Exception as exc:  # pylint: disable=broad-except
                    self.get_logger().error(f'Failed to reload costmap: {exc}')
                    return

        with self._lock:
            if self.metadata_template is None or self.costmap_data is None:
                self.get_logger().warn_throttle(5000, 'Costmap not yet loaded; skipping publish.')
                return

            now_msg = self.get_clock().now().to_msg()
            metadata = CostmapMetaData()
            metadata.resolution = self.metadata_template.resolution
            metadata.size_x = self.metadata_template.size_x
            metadata.size_y = self.metadata_template.size_y
            metadata.layer = self.metadata_template.layer
            metadata.map_load_time = self.map_load_time
            metadata.update_time = now_msg
            metadata.origin = self.metadata_template.origin

            msg = Costmap()
            msg.header.frame_id = self.map_frame
            msg.header.stamp = now_msg
            msg.metadata = metadata
            msg.data = list(self.costmap_data)

        self.publisher_.publish(msg)

        # publish equivalent nav_msgs/OccupancyGrid for RViz
        try:
            occ = OccupancyGrid()
            occ.header.frame_id = msg.header.frame_id
            occ.header.stamp = msg.header.stamp

            info = NavMapMetaData()
            info.resolution = metadata.resolution
            info.width = metadata.size_x
            info.height = metadata.size_y
            # reuse origin pose from CostmapMetaData
            info.origin = metadata.origin
            occ.info = info

            # map values: occupied -> 100, free -> 0, unknown -> -1
            mapped = []
            for v in msg.data:
                if v == self.occupied_value:
                    mapped.append(100)
                elif v == self.free_value:
                    mapped.append(0)
                else:
                    mapped.append(-1)
            occ.data = mapped

            self.occupancy_publisher_.publish(occ)
        except Exception:
            # don't crash the node if occupancy publish fails
            self.get_logger().debug('Failed to publish OccupancyGrid representation.')

    def destroy_node(self) -> bool:
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()
        return super().destroy_node()


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = PngCostmapPublisher()
    except Exception as exc:  # pylint: disable=broad-except
        rclpy.shutdown()
        raise exc

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
