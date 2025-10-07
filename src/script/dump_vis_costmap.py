import os
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt

class DumpVisualizeCostmap(Node):
    def __init__(self, topic, outdir, show):
        super().__init__('dump_visualize_costmap')
        self.topic = topic
        self.outdir = outdir
        self.show = show
        os.makedirs(self.outdir, exist_ok=True)
        self.sub = self.create_subscription(OccupancyGrid, self.topic, self.cb, 10)
        self.saved = False
        self.get_logger().info(f"订阅 {self.topic}，输出目录: {self.outdir}")

    def cb(self, msg: OccupancyGrid):
        if self.saved:
            return
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        arr = np.array(msg.data, dtype=np.int8)
        if arr.size != w * h:
            self.get_logger().error(f"数据长度与宽高不匹配: {arr.size} != {w}*{h}")
            return
        arr2d = arr.reshape((h, w))

        # 统计
        unique, counts = np.unique(arr2d, return_counts=True)
        stats = dict(zip(map(int, unique), map(int, counts)))
        self.get_logger().info(f"costmap 尺寸 {w}x{h}, 分辨率 {res}, origin ({ox},{oy})")
        self.get_logger().info(f"值分布: {stats}")

        # 保存原始数据
        npy_path = os.path.join(self.outdir, 'global_costmap.npy')
        csv_path = os.path.join(self.outdir, 'global_costmap.csv')
        np.save(npy_path, arr2d)
        np.savetxt(csv_path, arr2d, fmt='%d', delimiter=',')
        self.get_logger().info(f"已保存 numpy -> {npy_path} , csv -> {csv_path}")

        # 转换为灰度图并保存（-1->127 中灰, 0->255 白, 100->0 黑, 中间线性映射）
        image = np.zeros_like(arr2d, dtype=np.uint8)
        unknown_mask = (arr2d == -1)
        image[unknown_mask] = 127
        known = ~unknown_mask
        # 对已知值（0..100）做线性变换到 [255..0]
        vals = arr2d.astype(np.int16)
        vals_clipped = np.clip(vals, 0, 100)
        image[known] = (255 - (vals_clipped[known] * 255 // 100)).astype(np.uint8)

        img_path = os.path.join(self.outdir, 'global_costmap.png')
        plt.imsave(img_path, image, cmap='gray', vmin=0, vmax=255)
        self.get_logger().info(f"已保存图像 -> {img_path}")

        if self.show:
            plt.figure(figsize=(8,6))
            plt.title(f"Costmap ({w}x{h}) origin=({ox:.2f},{oy:.2f}) res={res}")
            plt.imshow(image, cmap='gray', origin='lower')
            plt.colorbar(label='value (mapped)')
            plt.show()

        self.saved = True
        # 关闭节点（脚本结束）
        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description='Dump and visualize OccupancyGrid costmap')
    parser.add_argument('--topic', '-t', default='/global_costmap/costmap', help='OccupancyGrid topic')
    parser.add_argument('--outdir', '-o', default='.', help='output directory')
    parser.add_argument('--show', action='store_true', help='show image window')
    args = parser.parse_args()

    rclpy.init()
    node = DumpVisualizeCostmap(args.topic, args.outdir, args.show)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()