#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Costmap Dump Visualizer

功能：
读取 `ros2 topic echo` 保存的 costmap.txt 文件，
将其解析还原为 OccupancyGrid 消息并发布，用于 RViz 可视化分析。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
import yaml
import os

class CostmapDumpPublisher(Node):
    def __init__(self):
        super().__init__('costmap_dump_publisher')
        
        # 声明参数
        self.declare_parameter('costmap_file', '/Pav-S_ws/src/costmap_process/costmap_chalu.txt')
        # self.declare_parameter('costmap_file', '/Pav-S_ws/src/costmap_process/costmap_huandao.txt')
        self.declare_parameter('topic_name', '/global_costmap/costmap_vis')
        self.declare_parameter('frame_id_override', '') 

        # 获取参数
        costmap_file = self.get_parameter('costmap_file').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id_override = self.get_parameter('frame_id_override').get_parameter_value().string_value
        
        # 检查文件
        if not os.path.exists(costmap_file):
            self.get_logger().error(f'Costmap文件不存在: {costmap_file}')
            raise FileNotFoundError(f'Costmap文件不存在: {costmap_file}')
        
        # 加载并解析数据
        self.get_logger().info(f'正在解析 Costmap 数据: {costmap_file} ...')
        try:
            self.costmap_msg = self.parse_costmap_txt(costmap_file)
            self.get_logger().info(
                f'解析成功! '
                f'尺寸: {self.costmap_msg.info.width}x{self.costmap_msg.info.height}, '
                f'分辨率: {self.costmap_msg.info.resolution}, '
                f'Frame: {self.costmap_msg.header.frame_id}'
            )
        except Exception as e:
            self.get_logger().error(f'解析失败: {e}')
            raise

        # QoS配置
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # 创建发布者
        self.pub = self.create_publisher(OccupancyGrid, topic_name, qos_profile)
        
        # 定时发布
        self.timer = self.create_timer(1.0, self.publish_costmap)
        self.get_logger().info(f'节点已启动，正在话题 {topic_name} 上发布...')
        
        # 立即发布一次
        self.publish_costmap()

    def parse_costmap_txt(self, file_path):
        """
        读取并解析 echo 的 txt 文件
        """
        msg = OccupancyGrid()
        
        with open(file_path, 'r', encoding='utf-8') as f:
            try:
                # === 关键修改：使用 safe_load_all 并获取第一个文档 ===
                # 解决 "expected a single document... but found another" 错误
                yaml_generator = yaml.safe_load_all(f)
                data_dict = next(yaml_generator)
            except StopIteration:
                raise ValueError("YAML 文件是空的")
            except yaml.YAMLError as exc:
                self.get_logger().error(f"YAML 解析错误: {exc}")
                raise

        # === 1. 填充 Header ===
        if 'header' in data_dict:
            header_data = data_dict['header']
            if self.frame_id_override:
                msg.header.frame_id = self.frame_id_override
            else:
                msg.header.frame_id = header_data.get('frame_id', 'map')
            
            # 使用当前时间戳
            msg.header.stamp = self.get_clock().now().to_msg()
        else:
            self.get_logger().warn("文件中未找到 header 字段，使用默认值")
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()

        # === 2. 填充 Info ===
        if 'info' in data_dict:
            info_data = data_dict['info']
            msg.info.resolution = float(info_data.get('resolution', 0.05))
            msg.info.width = int(info_data.get('width', 0))
            msg.info.height = int(info_data.get('height', 0))
            
            # Origin Position
            if 'origin' in info_data:
                origin_data = info_data['origin']
                if 'position' in origin_data:
                    msg.info.origin.position.x = float(origin_data['position'].get('x', 0.0))
                    msg.info.origin.position.y = float(origin_data['position'].get('y', 0.0))
                    msg.info.origin.position.z = float(origin_data['position'].get('z', 0.0))
                
                # Origin Orientation
                if 'orientation' in origin_data:
                    msg.info.origin.orientation.x = float(origin_data['orientation'].get('x', 0.0))
                    msg.info.origin.orientation.y = float(origin_data['orientation'].get('y', 0.0))
                    msg.info.origin.orientation.z = float(origin_data['orientation'].get('z', 0.0))
                    msg.info.origin.orientation.w = float(origin_data['orientation'].get('w', 1.0))
        else:
            raise ValueError("文件中缺少 'info' 字段，无法构建地图")

        # === 3. 填充 Data ===
        if 'data' in data_dict:
            raw_data = data_dict['data']
            if isinstance(raw_data, list):
                msg.data = [int(x) for x in raw_data]
            else:
                raise ValueError("data 字段格式不正确，应为列表")
        else:
            msg.data = [0] * (msg.info.width * msg.info.height)
            self.get_logger().warn("文件中未找到 data 字段，生成空地图")

        return msg

    def publish_costmap(self):
        self.costmap_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.costmap_msg)
        self.get_logger().debug('Published costmap from dump file.')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CostmapDumpPublisher()
        rclpy.spin(node)
    except (ValueError, FileNotFoundError) as e:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()