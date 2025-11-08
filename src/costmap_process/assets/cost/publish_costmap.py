#!/usr/bin/env python3
"""
publish_costmap.py

ROS2 rclpy 节点：读取 ./costmaps 下的图片文件（1_up.jpg, 1_down.jpg, 2_up.jpg, 2_down.jpg），
将其转换为 nav_msgs.msg.OccupancyGrid 并发布到话题 `processed_costmap`。

用法：
  ./publish_costmap.py [--state N]
其中 N = 1 (1_up), 2 (1_down), 3 (2_up), 4 (2_down)。

依赖：rclpy, nav_msgs, std_msgs, sensor_msgs, cv2, numpy

"""

import os
import sys
import argparse

# Minimal lazy import - rclpy may not be available in this environment; handle gracefully for syntax checks
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import Header
except Exception:
    rclpy = None
    Node = object
    OccupancyGrid = None
    Header = None

import cv2
import numpy as np


IMG_MAP = {
    1: 'costmaps/1_up.jpg',
    2: 'costmaps/1_down.jpg',
    3: 'costmaps/2_up.jpg',
    4: 'costmaps/2_down.jpg',
}


class CostmapPublisher(Node):
    def __init__(self, state=1, publish_rate=1.0):
        # If rclpy is unavailable, create a dummy initializer for syntax check
        if rclpy is not None:
            super().__init__('publish_costmap')
        else:
            # emulate attributes used below for safety when rclpy missing
            self.get_logger = lambda: None
        self.state = int(state)
        self.publish_rate = float(publish_rate)

        # Publisher
        if rclpy is not None:
            self.pub = self.create_publisher(OccupancyGrid, 'processed_costmap', 10)
        else:
            self.pub = None

        img_path = IMG_MAP.get(self.state)
        if img_path is None:
            raise ValueError('state must be 1..4')

        if not os.path.isabs(img_path):
            # relative to script cwd
            base = os.path.dirname(os.path.realpath(__file__))
            img_path = os.path.join(base, img_path)

        if not os.path.exists(img_path):
            raise FileNotFoundError(f"Image not found: {img_path}")

        self.img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            raise RuntimeError(f'Failed to load image: {img_path}')

        # Optionally flip vertical to match map origin depending on image conventions
        # We'll keep as-is but document it.

        if rclpy is not None:
            # Create a timer for periodic publishing
            self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def image_to_occupancy(self, image: np.ndarray) -> OccupancyGrid:
        # image: 2D uint8 grayscale
        h, w = image.shape
        # Define occupancy grid
        grid = OccupancyGrid()
        # Header
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg() if rclpy is not None else Header().stamp
        grid.header.frame_id = 'map'

        # Info
        grid.info.resolution = 1.0  # 1 meter per pixel - user should change as needed
        grid.info.width = int(w)
        grid.info.height = int(h)
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # Convert: assume white(255)=free(0), black(0)=occupied(100)
        # map 0..255 -> occupancy -1..100
        flat = image.flatten()
        occupancy = np.empty(flat.shape, dtype=np.int8)
        # threshold at 127
        occupancy[flat >= 128] = 0
        occupancy[flat < 128] = 100

        grid.data = list(occupancy.tolist())
        return grid

    def timer_callback(self):
        try:
            grid = self.image_to_occupancy(self.img)
            if self.pub is not None:
                self.pub.publish(grid)
                self.get_logger().info(f'Published occupancy grid for state={self.state}')
            else:
                print(f'[DEBUG] Would publish occupancy grid for state={self.state} (rclpy unavailable)')
        except Exception as e:
            if rclpy is not None:
                self.get_logger().error(f'Failed to publish: {e}')
            else:
                print('Error publishing:', e)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', type=int, default=1, choices=[1,2,3,4], help='state: 1..4')
    parser.add_argument('--rate', type=float, default=1.0, help='publish rate (Hz)')
    args = parser.parse_args(argv)

    if rclpy is None:
        print('rclpy not available in this environment. Script written for ROS2. Doing a dry-run syntax check.')
        # perform minimal checks: load image and convert
        base = os.path.dirname(os.path.realpath(__file__))
        img_path = os.path.join(base, IMG_MAP[args.state])
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print('Failed to load image for state', args.state)
            return 1
        # simulate conversion
        cp = CostmapPublisher(state=args.state, publish_rate=args.rate)
        grid = cp.image_to_occupancy(img)
        print('Dry-run successful. Occupancy grid size:', grid.info.width, 'x', grid.info.height)
        return 0

    rclpy.init()
    node = CostmapPublisher(state=args.state, publish_rate=args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
