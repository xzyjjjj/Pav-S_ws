"""
perception_mapping_node.py

(此文件重构自 combined.py)

主处理节点，功能包括：
1. (ROS模式) 使用 message_filters 同步 /rgb_img 和 /yolo_detections。
2. (LOCAL模式) 使用定时器读取本地数据文件。
3. 统一调用核心处理逻辑：
    a. 获取 TF 变换 (map -> body)。
    b. 分割图像 (FilterPoints)。
    c. 投影像素 (TfTransformer)。
    d. 绘制地图 (DrawMap)。
    e. 发布占据地图和调试图像。
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
import cv2
from pathlib import Path
import time

# ROS 消息和服务
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

# --- message_filters ---
import message_filters
try:
    from vision_msgs.msg import Detection2DArray
except Exception:
    Detection2DArray = None
    print("警告: 无法导入 'vision_msgs.msg.Detection2DArray'。")
    print("如果 'yolo_detections' 话题使用此类型，ROS 模式将失败。")

# 导入本地模块
from .config import *
from . import data_interface as data_utils # 重命名的工具模块
from .draw_map import DrawMap
from .tf_transformer import TfTransformer
from .filter_point import FilterPoints
from .extract_red_point import RedExtracter

class PerceptionMappingNode(Node):
    """
    ROS2 节点，整合了感知、投影、建图的完整管线。
    使用 message_filters 进行数据同步。
    """
    def __init__(self, node_name: str = 'pipeline_node'):
        super().__init__(node_name)
        self.get_logger().info(f"节点 '{node_name}' 开始初始化...")

        # 1. 初始化核心工具（只执行一次）
        self.bridge = CvBridge()
        self.drawer = DrawMap(MAP_ORIGIN_PATH)
        self.transformer = TfTransformer()
        self.filter = FilterPoints()
        self.frame_count = 0
        self.red_extracter = RedExtracter()

        # 2. 初始化 TF 监听器
        try:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        except Exception as e:
            self.get_logger().error(f"初始化 TF 监听器失败: {e}")
            self.tf_buffer = None
            self.tf_listener = None

        # 3. 初始化发布者
        self._debug_img_pub = self.create_publisher(Image, '/debug_drawn_img', 1)
        # 注意：占据地图的发布者在 self.drawer 内部，通过调用 publish_occupancy_from_png(node=self) 来使用

        # 4. 根据模式 (LOCAL vs ROS) 设置订阅或定时器
        if LOCAL:
            self.get_logger().info("运行在 LOCAL 模式。")
            # 在 LOCAL 模式下，我们使用定时器模拟数据流
            self.local_frame_index = 0
            # TODO: 从 config.py 或其他地方获取本地数据路径
            # 示例路径，你需要修改为你本地的路径
            self.local_data_dir = Path(LOCAL_DATA_PATH) 
            self.timer_period = 1.0 / LOCAL_SPIN_RATE_HZ # 假设 1Hz
            self.timer = self.create_timer(self.timer_period, self.local_mode_callback)
        else:
            self.get_logger().info("运行在 ROS 模式。")
            if Detection2DArray is None:
                self.get_logger().error("Detection2DArray 未导入，ROS 模式无法启动订阅。")
                return

            # --- 使用 message_filters ---
            # 1. 创建订阅者
            self.image_sub = message_filters.Subscriber(self, Image, 'rgb_img')
            self.detect_sub = message_filters.Subscriber(self, Detection2DArray, 'yolo_detections')
            
            # 再单独给rgb_img创建一个回调函数
            self.raw_image_sub = self.create_subscription(
                Image,
                'rgb_img',
                self.image_callback,
                10
            )

            # 2. 创建时间同步器 (使用近似时间同步)
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.detect_sub],
                queue_size=10,
                slop=0.2  # 允许 0.2 秒的时间差
            )
            
            # 3. 注册同步回调
            self.ts.registerCallback(self.ros_mode_callback)
            self.get_logger().info("Message_filters 同步已启动，等待 'rgb_img' 和 'yolo_detections'...")
            
            # (可选) 调试用：显示实时图像
            if SHOW_ORIGIN_MAP_WINDOW:
                self.create_subscription(Image, 'rgb_img', self._show_img_cb, 10)
    
    def image_callback(self, img_msg):
         # 单独处理 rgb_img（不会影响同步器）
        self.get_logger().info('收到独立的 rgb_img 消息')
        
        # 1. 解析图像
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            if img is None or img.shape[0] == 0 or img.shape[1] == 0:
                self.get_logger().warn("收到的图像为空或无效。")
                return
        except Exception as e:
            self.get_logger().error(f"CvBridge 转换图像失败: {e}")
            return


        # 3. 获取 TF 变换 (使用消息头的时间戳)
        T_map_to_body = self.get_tf_matrix('map', 'body', img_msg.header.stamp)
        if T_map_to_body is None:
            self.get_logger().warn("获取 TF(map -> body) 失败，跳过此帧。")
            return
            
        # 4. 
        masked_img = self.red_extracter.extract_red_points(img)
        non_black = np.any(masked_img != 0, axis=2)
        ys, xs = np.where(non_black)
        pts_2d = list(zip(xs, ys)) # (u, v) 像素坐标
        
        originmap_pixels = []
            
        # 使用 self.transformer 实例
        for u, v in pts_2d:
            try:
                res = self.transformer.calc_point_on_origin_map((u, v), T_map_to_body, count=0) # count 参数似乎没用，保持为0
                pixel = res.get('map_pixel')
                if pixel is not None:
                    originmap_pixels.append(pixel)
            except Exception as e:
                self.get_logger().warn(f"calc_point_on_origin_map 失败: {e}")
                continue
            
        self.get_logger().info(f"类型 'red_zone / red_cone' 投影了 {len(originmap_pixels)} 个点。")

        # 3. 绘制地图并发布
        any_drawn = False
        drawn_img_for_debug = None
            
        color_info = OCCUPANCY_COLOR_MAP.get('red_zone') # 红色的东西全当 red_zone 去做
        rgb = tuple(int(x) for x in color_info['rgb'])
            
        try:
            # 使用 self.drawer 实例
            drawn_img_for_debug = self.drawer.draw_point_on_map(
                originmap_pixels, 
                color=rgb, 
                radius=2
            )
            any_drawn = True
            
        except Exception as e:
            self.get_logger().warn(f"draw_point_on_map 失败 (type red_zone / red_cone): {e}")

        # 4. 发布调试图和占据地图
        if any_drawn:
            self.get_logger().info("绘制了新点，发布调试图像和占据地图...")
            # 发布 debug 绘制结果
            try:
                if drawn_img_for_debug is not None:
                    msg = self.bridge.cv2_to_imgmsg(drawn_img_for_debug, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self._debug_img_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"发布 debug drawn_img 失败: {e}")

            # 发布占据地图
            try:
                # 将 self (节点实例) 传递给发布函数
                self.drawer.publish_occupancy_from_png(node=self, topic=None, show=False)
            except Exception as e:
                self.get_logger().warn(f"publish_occupancy_from_png 失败: {e}")
        else:
            self.get_logger().info("没有新的点被绘制。")



    def _show_img_cb(self, msg: 'Image'):
        """调试回调：收到 rgb_img 时弹窗显示"""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if img is not None and hasattr(img, 'shape') and img.shape[0] > 0:
                cv2.imshow('rgb_img_realtime', img)
                cv2.waitKey(1) # 必须是 waitKey(1) 否则会阻塞
        except Exception as e:
            self.get_logger().warn(f"[Pipeline] imshow failed in _show_img_cb: {e}")

    def get_tf_matrix(self, target_frame: str, source_frame: str, timestamp: 'Time') -> np.ndarray:
        """
        查找指定时间戳的 TF 变换并返回 4x4 矩阵。
        如果失败，返回 None。
        """
        if self.tf_buffer is None:
            self.get_logger().warn("TF buffer 未初始化。")
            return None
        
        try:
            # 等待变换可用，设置短暂超时
            if not self.tf_buffer.can_transform(target_frame, source_frame, timestamp, timeout=rclpy.duration.Duration(seconds=0.1)):
                self.get_logger().warn(
                    f"无法变换 {source_frame} -> {target_frame} at time {timestamp.nanosec}"
                )
                return None

            t = self.tf_buffer.lookup_transform(target_frame, source_frame, timestamp)
            
            tr = t.transform.translation
            rot = t.transform.rotation
            tr_vec = [float(tr.x), float(tr.y), float(tr.z)]
            quat_vec = [float(rot.x), float(rot.y), float(rot.z), float(rot.w)]
            
            return data_utils.tf_matrix_from_tr_and_quat(tr_vec, quat_vec)

        except Exception as e:
            self.get_logger().error(f"查找 TF ({source_frame} -> {target_frame}) 失败: {e}")
            return None

    def ros_mode_callback(self, img_msg: 'Image', det_msg: 'Detection2DArray'):
        """
        message_filters 的同步回调函数。
        """
        self.frame_count += 1
        self.get_logger().info(f"--- [ROS] 收到同步帧 {self.frame_count} ---")
        
        # 1. 解析图像
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            if img is None or img.shape[0] == 0 or img.shape[1] == 0:
                self.get_logger().warn("收到的图像为空或无效。")
                return
        except Exception as e:
            self.get_logger().error(f"CvBridge 转换图像失败: {e}")
            return

        # 2. 解析检测结果
        # 使用 data_utils 中的辅助函数
        bboxes_list = data_utils.parse_bbox_from_obj(det_msg)
        if not bboxes_list:
            self.get_logger().info("当前帧没有检测到目标。")
            # 即使没有目标，也可以选择继续处理（例如清除地图），或者直接返回
            # return 

        # 3. 获取 TF 变换 (使用消息头的时间戳)
        T_map_to_body = self.get_tf_matrix('map', 'body', img_msg.header.stamp)
        if T_map_to_body is None:
            self.get_logger().warn("获取 TF(map -> body) 失败，跳过此帧。")
            return
            
        # 4. 调用核心处理逻辑
        self.process_data(img, bboxes_list, T_map_to_body)

    def local_mode_callback(self):
        """
        LOCAL 模式下的定时器回调函数。
        """
        self.frame_count += 1
        self.get_logger().info(f"--- [LOCAL] 处理帧 {self.frame_count} (index {self.local_frame_index}) ---")

        # 构造文件名 (假设格式为 000000_img.png, 000000_detections.json, 000000_tf.json)
        # 你需要根据你的实际文件名格式修改这里
        base_name = f"{self.local_frame_index:06d}"
        img_path = self.local_data_dir / f"{base_name}_img.png" # 假设图像是png
        bbox_path = self.local_data_dir / f"{base_name}_detections.json"
        tf_path = self.local_data_dir / f"{base_name}_tf.json"

        # 1. 加载数据
        if not img_path.exists() or not bbox_path.exists() or not tf_path.exists():
            self.get_logger().error(f"本地数据文件缺失 (index {self.local_frame_index})，停止定时器。")
            self.timer.cancel()
            return
            
        img = cv2.imread(str(img_path))
        if img is None:
            self.get_logger().error(f"读取本地图像失败: {img_path}")
            return
            
        bboxes_list = data_utils.read_bbox(str(bbox_path))
        T_map_to_body = data_utils.read_tf(str(tf_path))

        # 2. 调用核心处理逻辑
        self.process_data(img, bboxes_list, T_map_to_body)
        
        # 索引递增，用于下一帧
        self.local_frame_index += 1

    def process_data(self, img: np.ndarray, bboxes_list: list, T_map_to_body: np.ndarray):
        """
        核心处理管线，被 ROS 回调和 Local 回调共同调用。
        """
        self.get_logger().info(f"Processing frame {self.frame_count} with {len(bboxes_list)} detections.")
        
        # 1. (原 process_frame) 分割出所有感兴趣的区域
        masked_imgs = []
        types = data_utils.get_all_type(bboxes_list)

        for type_m in types:
            pts_corners = data_utils.get_specific_bbox(type_m, bboxes_list)
            bboxes_int = []
            if pts_corners:
                bboxes_int = [[int(round(p[0])), int(round(p[1]))] for p in pts_corners]
            
            # 使用 self.filter 实例
            masked_img, _type = self.filter.extract_points(img, bboxes_int if bboxes_int else None, type_m)
            masked_imgs.append((masked_img, _type))
            
            if SHOW_ORIGIN_MAP_WINDOW:
                try:
                    cv2.imshow(f'masked_{_type}', masked_img)
                    cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().warn(f"[process_data] 弹窗显示 masked_img 失败: {e}")

        # 2. (原 run_once 后半段) 投影像素
        maps_data = [] # 存储 (originmap_pixels, type_m)

        for img_m, type_m in masked_imgs:
            non_black = np.any(img_m != 0, axis=2)
            ys, xs = np.where(non_black)
            if len(xs) == 0:
                continue
            
            pts_2d = list(zip(xs, ys)) # (u, v) 像素坐标
            originmap_pixels = []
            
            # 使用 self.transformer 实例
            for u, v in pts_2d:
                try:
                    res = self.transformer.calc_point_on_origin_map((u, v), T_map_to_body, count=0) # count 参数似乎没用，保持为0
                    pixel = res.get('map_pixel')
                    if pixel is not None:
                        originmap_pixels.append(pixel)
                except Exception as e:
                    self.get_logger().warn(f"calc_point_on_origin_map 失败: {e}")
                    continue
            
            self.get_logger().info(f"类型 '{type_m}' 投影了 {len(originmap_pixels)} 个点。")
            maps_data.append((originmap_pixels, type_m))

        # 3. 绘制地图并发布
        any_drawn = False
        drawn_imgs_for_debug = []
        
        for _originmap_pixels, type_m in maps_data:
            if not _originmap_pixels:
                continue

            # 不用这个逻辑处理red_zone和red_cone
            if type_m == 'red_zone' or type_m == 'red_cone': 
                continue
            
            color_info = OCCUPANCY_COLOR_MAP.get(str(type_m)) or OCCUPANCY_COLOR_MAP.get(type_m)
            if color_info and 'rgb' in color_info:
                rgb = tuple(int(x) for x in color_info['rgb'])
            else:
                rgb = (0, 0, 255)  # 默认红色 (RGB)
            
            try:
                # 使用 self.drawer 实例
                drawn_imgs_for_debug.append( self.drawer.draw_point_on_map(
                    _originmap_pixels, 
                    color=rgb, 
                    radius=2
                ) )
                any_drawn = True
                
            except Exception as e:
                self.get_logger().warn(f"draw_point_on_map 失败 (type {type_m}): {e}")

        # 4. 发布调试图和占据地图
        if any_drawn:
            self.get_logger().info("绘制了新点，发布调试图像和占据地图...")
            # 发布 debug 绘制结果
            try:
                if drawn_imgs_for_debug is not None:
                    for img in drawn_imgs_for_debug:
                        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                        msg.header.stamp = self.get_clock().now().to_msg()
                        self._debug_img_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"发布 debug drawn_img 失败: {e}")

            # 发布占据地图
            try:
                # 将 self (节点实例) 传递给发布函数
                self.drawer.publish_occupancy_from_png(node=self, topic=None, show=False)
            except Exception as e:
                self.get_logger().warn(f"publish_occupancy_from_png 失败: {e}")
        else:
            self.get_logger().info("没有新的点被绘制。")


def main(args=None):
    rclpy.init(args=args)
    try:
        pipeline_node = PerceptionMappingNode()
        rclpy.spin(pipeline_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行时发生未捕获异常: {e}")
    finally:
        if 'pipeline_node' in locals() and rclpy.ok():
            pipeline_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()