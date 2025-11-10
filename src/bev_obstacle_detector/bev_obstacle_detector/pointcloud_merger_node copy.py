import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

# TF2 库用于坐标系变换
import tf2_ros
from tf2_ros import TransformException

import numpy as np

class PointCloudMerger(Node):
    
    def __init__(self):
        super().__init__('pointcloud_merger_node')
        
        self.get_logger().info('PointCloudMerger Node has started.')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_topic_lidar', '/cloud_registered'),
                ('input_topic_bev', '/bev/obstacles'),
                ('output_topic_fused', '/fused_pointcloud'),
                ('target_frame', 'body'),
                ('tf_timeout_seconds', 0.05),
                ('fusion_time_tolerance', 0.01)
            ]
        )

        self.input_topic_lidar = self.get_parameter('input_topic_lidar').value
        self.input_topic_bev = self.get_parameter('input_topic_bev').value
        self.output_topic_fused = self.get_parameter('output_topic_fused').value
        self.target_frame = self.get_parameter('target_frame').value
        self.tf_timeout_seconds = self.get_parameter('tf_timeout_seconds').value
        self.fusion_time_tolerance = self.get_parameter('fusion_time_tolerance').value


        # --- 1. 初始化 TF 监听器 ---
        # 用于监听 body -> base_link 的变换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- 2. 初始化点云存储 ---
        # 存储最近收到的两个点云，确保它们在同一时间戳附近
        self.bev_pc_msg = None
        self.lidar_pc_msg = None
        
        # --- 3. 初始化 ROS 接口 ---
        
        # 订阅 BEV 障碍物点云 (已在 base_link)
        self.create_subscription(
            PointCloud2, 
            self.input_topic_bev,
            self.bev_callback, 
            10
        )
        
        # 订阅 Lidar 原始点云 (在 body 坐标系)
        self.create_subscription(
            PointCloud2, 
            self.input_topic_lidar,
            self.lidar_callback, 
            10
        )
        
        # 发布融合后的点云
        self.fused_publisher = self.create_publisher(
            PointCloud2, 
            self.output_topic_fused,
            10
        )
        self.get_logger().info(f"Fusion Node publishing to: {self.output_topic_fused}")

    def bev_callback(self, msg: PointCloud2):
        """处理 BEV 点云的回调函数"""
        # BEV 点云已经在目标坐标系 base_link 下，直接存储
        self.bev_pc_msg = msg
        self.try_fuse_pointclouds()

    def lidar_callback(self, msg: PointCloud2):
        """处理 Lidar 点云的回调函数"""
        self.lidar_pc_msg = msg
        self.try_fuse_pointclouds()

    def try_fuse_pointclouds(self):
        """尝试融合两个点云"""
        # 确保两个传感器都有数据
        if self.bev_pc_msg is None or self.lidar_pc_msg is None:
            return

        # 优先使用 LiDAR 的时间戳作为融合点云的基准时间
        fusion_stamp = self.lidar_pc_msg.header.stamp
        
        # 1. 尝试将 LiDAR 点云从 'body' 变换到 'base_link'
        transformed_lidar_pc = self.transform_pointcloud(
            self.lidar_pc_msg, 
            target_frame='base_link', 
            source_frame=self.lidar_pc_msg.header.frame_id
        )

        if transformed_lidar_pc is None:
            # 变换失败，等待下一个 TF 变换或点云
            self.get_logger().warn("变换失败，等待下一个 TF 变换或点云")
            return

        # 2. 提取两个点云的 (x, y, z) 坐标
        # BEV 点云已经是 base_link 坐标系
        bev_points = self.extract_points(self.bev_pc_msg)
        lidar_points = self.extract_points(transformed_lidar_pc)
        
        if len(lidar_points) == 0:
            self.get_logger().warn("Lidar点云为空，跳过融合。")
            return

        # 3. 合并点云 (NumPy 数组)
        # 注意：这里我们假设 BEV 点云不会包含太多的点，以避免巨大的内存开销
        if len(bev_points) > 0:
            fused_points_np = np.concatenate((lidar_points, bev_points), axis=0)
        else:
            fused_points_np = lidar_points
        
        # 4. 创建并发布融合后的 PointCloud2 消息
        self.get_logger().warn("成功创建并发布融合后的 PointCloud2 消息")
        fused_header = Header(
            stamp=fusion_stamp, 
            frame_id='base_link'
        )
        
        # 定义点云的字段 (只包含 x, y, z)
        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1)
        ]
        
        # 将 NumPy 数组转换回 PointCloud2 消息
        fused_pc_msg = point_cloud2.create_cloud(
            fused_header, 
            fields, 
            fused_points_np.tolist()
        )
        
        self.fused_publisher.publish(fused_pc_msg)
        
        # 融合完成后，清空 BEV 点云，等待下一个新的 BEV 帧
        # Lidar 点云保留，直到下一个 Lidar 帧到达
        self.bev_pc_msg = None


    def transform_pointcloud(self, pc_msg: PointCloud2, target_frame: str, source_frame: str) -> PointCloud2:
        """使用 tf2_ros 变换点云坐标系"""
        if target_frame == source_frame:
            return pc_msg
            
        try:
            # 等待变换可用
            self.tf_buffer.can_transform(
                target_frame, 
                source_frame, 
                pc_msg.header.stamp, 
                timeout=rclpy.duration.Duration(self.tf_timeout_seconds)
            )
            # 执行变换
            pc_transformed = self.tf_buffer.transform(pc_msg, target_frame)
            return pc_transformed
            
        except TransformException as ex:
            self.get_logger().warn(f'TF 变换失败 ({source_frame} -> {target_frame}): {ex}')
            return None


    def extract_points(self, pc_msg: PointCloud2) -> np.ndarray:
        """从 PointCloud2 消息中提取 (x, y, z) 数组"""
        
        # 使用 sensor_msgs_py 库的 read_points 简化提取
        points_generator = point_cloud2.read_points(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points_list = list(points_generator)
        
        if not points_list:
            return np.empty((0, 3), dtype=np.float32)
            
        return np.array(points_list, dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()