# 写屎了 别管

#!/usr/bin/env python3
import argparse
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
try:
    from nav2_msgs.msg import Costmap as Nav2Costmap  # optional
except Exception:
    Nav2Costmap = None


class CostmapProcessor(Node):
    def __init__(self, topic_in, topic_out, line_coords, thickness):
        super().__init__('process_costmap')
        self.topic_in = topic_in
        self.topic_out = topic_out
        self.line_coords = line_coords  # (x1,y1,x2,y2) in world meters
        self.thickness = thickness  # meters

        # 订阅 nav2_msgs/Costmap
        self.sub = self.create_subscription(
            Nav2Costmap if Nav2Costmap is not None else OccupancyGrid,
            self.topic_in, self.cb, 10)
        # 发布为 OccupancyGrid
        self.pub = self.create_publisher(OccupancyGrid, self.topic_out, 1)
        self.get_logger().info(f"Subscribed {self.topic_in} -> publishing processed to {self.topic_out}")

    def cb(self, msg):
        # 兼容 nav2_msgs/Costmap 消息结构
        try:
            # 解析 metadata 字段
            w = msg.metadata.size_x
            h = msg.metadata.size_y
            res = msg.metadata.resolution
            ox = msg.metadata.origin.position.x
            oy = msg.metadata.origin.position.y
            # 数据
            data = np.array(msg.data, dtype=np.int16)
            if data.size != w * h:
                self.get_logger().error(f"data length mismatch: {data.size} != {w}*{h}")
                return
            arr = data.reshape((h, w))

            # 修正数据类型和数值范围
            arr[arr > 100] = -1  # 超过100的设为未知
            arr[arr < -1] = -1   # 小于-1的也设为未知
            arr = arr.astype(np.int8)

            # 世界坐标转栅格坐标
            x1, y1, x2, y2 = self.line_coords
            cx1 = (x1 - ox) / res
            cy1 = (y1 - oy) / res
            cx2 = (x2 - ox) / res
            cy2 = (y2 - oy) / res

            self._draw_thick_line(arr, cx1, cy1, cx2, cy2, int(np.ceil(self.thickness / res)), value=100)

            # 构造 OccupancyGrid 消息
            from nav_msgs.msg import OccupancyGrid
            from geometry_msgs.msg import Pose
            grid_msg = OccupancyGrid()
            grid_msg.header = msg.header
            grid_msg.info.resolution = res
            grid_msg.info.width = w
            grid_msg.info.height = h
            grid_msg.info.origin = Pose()
            grid_msg.info.origin.position.x = ox
            grid_msg.info.origin.position.y = oy
            grid_msg.info.origin.position.z = msg.metadata.origin.position.z
            grid_msg.info.origin.orientation = msg.metadata.origin.orientation
            grid_msg.data = [int(x) for x in arr.flatten()]

            self.pub.publish(grid_msg)
            self.get_logger().info("Published processed costmap")
        except Exception as e:
            self.get_logger().error(f"Failed to process Costmap msg: {e}")

    @staticmethod
    def _draw_thick_line(arr, x0, y0, x1, y1, thickness_cells, value=100):
        # Bresenham-like sampling + disk brush
        dx = x1 - x0
        dy = y1 - y0
        length = int(max(1, np.hypot(dx, dy)))
        for i in range(length + 1):
            t = i / max(1, length)
            cx = int(round(x0 + dx * t))
            cy = int(round(y0 + dy * t))
            # brush
            r = thickness_cells // 2
            for oy in range(-r, r + 1):
                yy = cy + oy
                if yy < 0 or yy >= arr.shape[0]:
                    continue
                for ox in range(-r, r + 1):
                    xx = cx + ox
                    if xx < 0 or xx >= arr.shape[1]:
                        continue
                    arr[yy, xx] = value


def parse_line_arg(s):
    parts = s.split(',')
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("line must be x1,y1,x2,y2")
    return tuple(float(p) for p in parts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--in', '-i', dest='topic_in', default='/global_costmap/costmap_raw')
    parser.add_argument('--out', '-o', dest='topic_out', default='/global_costmap/costmap_raw_processed')
    parser.add_argument('--line', '-l', dest='line', type=parse_line_arg, default=(1.0, 1.0, 2.0, 1.0),
                        help='line as x1,y1,x2,y2 (world meters)')
    parser.add_argument('--thickness', '-t', dest='thickness', type=float, default=0.05,
                        help='thickness in meters')
    args = parser.parse_args()

    rclpy.init()
    node = CostmapProcessor(args.topic_in, args.topic_out, args.line, args.thickness)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
