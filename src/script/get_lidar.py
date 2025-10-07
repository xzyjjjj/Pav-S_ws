import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
import open3d as o3d
import numpy as np

class SaveCustomPointCloud(Node):
    def __init__(self):
        super().__init__('save_custom_pointcloud')
        self.sub = self.create_subscription(CustomMsg, '/livox/lidar', self.callback, 10)
        self.saved = False

    def callback(self, msg):
        if self.saved:
            return
        points = []
        for pt in msg.points:
            points.append([pt.x, pt.y, pt.z])
        if len(points) == 0:
            self.get_logger().info("未收到点云数据")
            return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        o3d.io.write_point_cloud("livox_cloud.pcd", pcd)
        self.get_logger().info("点云已保存为 livox_cloud.pcd")
        self.saved = True

def main(args=None):
    rclpy.init(args=args)
    node = SaveCustomPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()