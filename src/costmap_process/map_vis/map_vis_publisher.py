#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Map Visualization Publisher (Fixed Logic & Fixed Frame ID)
1. 修复了 Alpha 通道混合逻辑（解决白背景变绿的问题）。
2. 修复了 frame_id 为空导致 RViz 无法显示的问题。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
import yaml
import numpy as np
from PIL import Image
import os
from enum import Enum
import math


# 常量定义
OCC_GRID_UNKNOWN = -1
OCC_GRID_FREE = 0
OCC_GRID_OCCUPIED = 100


class MapMode(Enum):
    Trinary = "trinary"
    Scale = "scale"
    Raw = "raw"


class MapVisPublisher(Node):
    def __init__(self):
        super().__init__('map_vis_publisher')
        
        self.declare_parameter('yaml_filename', '')
        self.declare_parameter('topic_name', '/map_vis')
        self.declare_parameter('frame_id', 'map')
        
        yaml_filename = self.get_parameter('yaml_filename').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        if not yaml_filename:
            self.get_logger().error('yaml_filename 参数未设置！')
            raise ValueError('yaml_filename 参数必须设置')
        
        if not os.path.exists(yaml_filename):
            self.get_logger().error(f'地图文件不存在: {yaml_filename}')
            raise FileNotFoundError(f'地图文件不存在: {yaml_filename}')
        
        self.get_logger().info(f'正在加载地图: {yaml_filename}')
        try:
            self.map_msg = self.load_map_from_yaml(yaml_filename, frame_id)
            self.get_logger().info(
                f'地图加载成功: {self.map_msg.info.width}x{self.map_msg.info.height}, '
                f'分辨率: {self.map_msg.info.resolution}m/cell, Frame: {frame_id}'
            )
        except Exception as e:
            self.get_logger().error(f'加载地图失败: {e}')
            raise
        
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.map_pub = self.create_publisher(OccupancyGrid, topic_name, qos_profile)
        self.get_logger().info(f'地图发布节点已启动，发布到话题: {topic_name}')
        
        self.timer = self.create_timer(1.0, self.publish_map)
        self.publish_map()
    
    def load_map_from_yaml(self, yaml_file: str, frame_id: str) -> OccupancyGrid:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f)
        
        image_file_name = doc['image']
        if not image_file_name:
            raise ValueError("The image tag was empty.")
        
        if image_file_name[0] != '/':
            yaml_dir = os.path.dirname(os.path.abspath(yaml_file))
            image_file_name = os.path.join(yaml_dir, image_file_name)
        
        resolution = float(doc['resolution'])
        origin = doc['origin']
        free_thresh = float(doc.get('free_thresh', 0.196))
        occupied_thresh = float(doc.get('occupied_thresh', 0.65))
        
        mode_str = doc.get('mode', 'trinary').lower()
        mode = MapMode.Scale if mode_str == 'scale' else (MapMode.Raw if mode_str == 'raw' else MapMode.Trinary)
        
        negate = doc.get('negate', 0)
        negate = 1 if (isinstance(negate, bool) and negate) or (isinstance(negate, int) and negate == 1) else 0
        
        if not os.path.exists(image_file_name):
            raise FileNotFoundError(f'地图图像文件不存在: {image_file_name}')
        
        img = Image.open(image_file_name)
        img_array = np.array(img)
        
        map_msg = OccupancyGrid()
        # === [Fix] 之前漏掉了这行，导致 RViz 报错 frame empty ===
        map_msg.header.frame_id = frame_id 
        
        map_msg.info.resolution = resolution
        map_msg.info.width = img_array.shape[1]
        map_msg.info.height = img_array.shape[0]
        map_msg.info.origin.position.x = float(origin[0])
        map_msg.info.origin.position.y = float(origin[1])
        map_msg.info.origin.position.z = float(origin[2])
        
        yaw = float(origin[2])
        map_msg.info.origin.orientation.z = math.sin(yaw / 2.0)
        map_msg.info.origin.orientation.w = math.cos(yaw / 2.0)
        
        # 预分配内存
        total_pixels = map_msg.info.width * map_msg.info.height
        map_msg.data = [0] * total_pixels
        
        # 图像属性判断
        has_alpha = False
        is_rgb = False
        if len(img_array.shape) == 3:
            if img_array.shape[2] == 3:
                is_rgb = True
            elif img_array.shape[2] == 4:
                is_rgb = True
                has_alpha = True
        
        # === 核心修正逻辑 (向量化) ===
        
        # 1. 提取 RGB 和 Alpha
        if is_rgb:
            rgb = img_array[:, :, :3].astype(float)
            # 仅计算 RGB 均值
            avg_rgb = np.mean(rgb, axis=2)
            
            if has_alpha:
                alpha = img_array[:, :, 3].astype(float) / 255.0
            else:
                alpha = np.ones_like(avg_rgb)
        else:
            # 灰度图
            avg_rgb = img_array.astype(float)
            alpha = np.ones_like(avg_rgb)

        # 2. 计算 Shade (Alpha Blending: RGB*alpha + White*(1-alpha))
        rgb_shade = avg_rgb / 255.0
        final_shade = (rgb_shade * alpha) + (1.0 * (1.0 - alpha))
        
        # 3. 计算 Occupancy
        if negate:
            occ = final_shade
        else:
            occ = 1.0 - final_shade
            
        # 4. 根据 Mode 进行分类
        data_flat = np.full(total_pixels, OCC_GRID_UNKNOWN, dtype=np.int8)
        occ_flat = occ.flatten()
        alpha_flat = alpha.flatten()
        
        if mode == MapMode.Trinary:
            occupied_mask = occ_flat > occupied_thresh
            free_mask = occ_flat < free_thresh
            data_flat[occupied_mask] = OCC_GRID_OCCUPIED
            data_flat[free_mask] = OCC_GRID_FREE
            
        elif mode == MapMode.Scale:
            unknown_mask = alpha_flat < 0.5
            data_flat[unknown_mask] = OCC_GRID_UNKNOWN
            
            with np.errstate(divide='ignore', invalid='ignore'):
                scale_vals = ((occ_flat - free_thresh) / (occupied_thresh - free_thresh)) * 100.0
            scale_vals = np.round(scale_vals).astype(int)
            scale_vals = np.clip(scale_vals, 0, 100)
            
            valid_mask = ~unknown_mask
            occ_mask = (occ_flat > occupied_thresh) & valid_mask
            free_mask = (occ_flat < free_thresh) & valid_mask
            mid_mask = (~occ_mask) & (~free_mask) & valid_mask
            
            data_flat[occ_mask] = OCC_GRID_OCCUPIED
            data_flat[free_mask] = OCC_GRID_FREE
            data_flat[mid_mask] = scale_vals[mid_mask]

        elif mode == MapMode.Raw:
            raw_vals = np.round(final_shade.flatten() * 255).astype(int)
            data_flat[:] = raw_vals
            
        # 5. Y轴翻转处理
        data_reshaped = data_flat.reshape((map_msg.info.height, map_msg.info.width))
        data_flipped = np.flipud(data_reshaped)
        
        map_msg.data = data_flipped.flatten().tolist()
        
        return map_msg
    
    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)
        self.get_logger().debug('已发布地图到 /map_vis')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MapVisPublisher()
        rclpy.spin(node)
    except (ValueError, FileNotFoundError) as e:
        rclpy.get_logger().error(f'启动失败: {e}')
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    exit(main())