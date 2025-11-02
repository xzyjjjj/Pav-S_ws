"""
combined.py

提供一个面向对象的工具，把相机坐标系下的单点通过给定的变换矩阵转换到 map 坐标系并投影到 origin map 图像像素。

用法示例：
    transformer = TfTransformer(map_img_path='red_zone_proj_assets/map_origin.jpg', map_range=((0,0),(5,3)))
    map_xyz, (u,v) = transformer.camera_point_to_originmap([0,0,0.6], T_MAP_TO_CAMERA)

约定：传入的 transform_matrix 应该是 T_MAP_TO_CAMERA（从 map -> camera 的 4x4 齐次矩阵），
函数会在内部取逆把相机点变换到 map。
"""

from pathlib import Path
import numpy as np
from numpy.linalg import inv
import cv2
from .draw_map import DrawMap
from .cal_xyz import calculate_xyz
import json
import logging
from pathlib import Path
import json
import threading
import time
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import Image
try:
    from vision_msgs.msg import Detection2DArray
except Exception:
    Detection2DArray = None
from .tf_transformer import TfTransformer
from .filter_point import FilterPoints
from .config import *
import rclpy
from rclpy.node import Node

def process_frame(fi, img, dets_obj, T_map_to_body, count):
    all_bboxes = fi.parse_bbox_from_obj(dets_obj)

    print(f"=======================round: {count}=======================")


    # 考虑所有检测目标
    types = fi.get_all_type(all_bboxes)
    print("进入 process_frame")
    print(f"拿到这些变量:{types}")
    print(f"dets_obj: {dets_obj}")
    print(f"T_map_to_body:{T_map_to_body}")
    print(f"all_bboxes:{all_bboxes}")
    masked_imgs = []

    for type in types:
        pts = fi.get_specific_bbox(type, all_bboxes)
        print(f"pts:{pts}")
        bboxes = []
        if pts:
            bboxes = [[int(round(p[0])), int(round(p[1]))] for p in pts]

        # 2) 用 extract_red_points 得到处理后的 rgb 图像
        filter = FilterPoints()
        # print("======================")
        # print(f"输入的检测框:{bboxes}")
        masked_img, _type = filter.extract_points(img, bboxes if bboxes else None, type)
        masked_imgs.append([masked_img, _type])
        # 弹窗显示 masked_img，阻塞直到用户按键关闭
        try:
            import cv2
            print("now showing maskex_img")
            cv2.imshow(f'masked_{type}', masked_img)
            cv2.waitKey(0)
            cv2.destroyWindow(f'masked_{type}')
        except Exception as e:
            print(f"[process_frame] 弹窗显示 masked_img 失败: {e}")

        # 显示分割结果（弹窗） — 只有在 SHOW_ORIGIN_MAP_WINDOW 且 NOT LOCAL 时显示
        if SHOW_ORIGIN_MAP_WINDOW and not LOCAL:
            cv2.imshow(f'masked_{type}', masked_img)
            cv2.waitKey(0)
            cv2.destroyWindow(f'masked_{type}')
            pass
    print("out process_frame")
    return masked_imgs

class PipelineNode(Node):
    """ROS2 Node: pipeline for segmentation->projection->draw->publish."""
    def __init__(self, node_name: str = 'pipeline_node', spin_rate_hz: float = 10.0):
        super().__init__(node_name)
        # Inline minimal DataInterface behavior here so PipelineNode is self-contained.
        self._bridge = CvBridge()
        # storage for last messages
        self._last_image = None
        self._last_detections = None
        self._image_event = threading.Event()
        self._det_event = threading.Event()
        # TF support
        try:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        except Exception:
            self._tf_buffer = None
            self._tf_listener = None

        # subscriptions
        try:
            self.create_subscription(Image, 'rgb_img', self._img_cb, 10)
            if Detection2DArray is not None:
                self.create_subscription(Detection2DArray, 'yolo_detections', self._det_cb, 10)
            # 额外订阅 rgb_img 并弹窗显示
            self.create_subscription(Image, 'rgb_img', self._show_img_cb, 10)
        except Exception:
            pass
        self.loop_timeout = 2.0
        self.drawer = DrawMap(MAP_ORIGIN_PATH)
        self.timer_period = 1.0 / spin_rate_hz
        self.timer = self.create_timer(self.timer_period, self.run_once)
        self.count = 0
    def _show_img_cb(self, msg: 'Image'):
        """收到 rgb_img 时弹窗显示"""
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            try:
                img = self._bridge.imgmsg_to_cv2(msg)
            except Exception:
                img = None
        if img is not None and hasattr(img, 'shape') and img.shape[0] > 0 and img.shape[1] > 0:
            try:
                # cv2.imshow('rgb_img_sub', img)
                # cv2.waitKey(0)
                pass
            except Exception as e:
                self.get_logger().warn(f"[Pipeline] imshow failed in _show_img_cb: {e}")

    def run_once(self):
        try:
            # 改回调用 data_interface.py 的 DataInterface 实例方法
            from .data_interface import DataInterface
            if not hasattr(self, 'di'):
                self.di = DataInterface()
            img, dets_obj, T_map_to_body = self.di.get_topic_data()
            # 用 _show_img_cb 获得的 img 替换（仅当有效且 shape 正确时才覆盖）
            if hasattr(self, '_last_img_from_show_cb') and self._last_img_from_show_cb is not None:
                cb_img = self._last_img_from_show_cb
                if hasattr(cb_img, 'shape') and cb_img.shape[0] > 0 and cb_img.shape[1] > 0:
                    img = cb_img
            print(f"img is {img == True}")
        except Exception as e:
            self.get_logger().warn(f'[Pipeline] get_topic_data error: {e}')
            return
        try:
            import cv2
            cv2.imshow(f'img', img)
            cv2.waitKey(0)
            cv2.destroyWindow(f'img')
        except Exception as e:
            print(f"[process_frame] 弹窗显示 img 失败: {e}")
        try:
            if img is None:
                print("[Pipeline] imshow skipped: img is None")
            elif not hasattr(img, 'shape') or img.shape[0] == 0 or img.shape[1] == 0:
                print(f"[Pipeline] imshow skipped: invalid img shape: {getattr(img,'shape',None)}")
            else:
                cv2.imshow('input_img', img)
                cv2.waitKey(0)  # or appropriate waitKey(ms)
                cv2.destroyWindow('input_img')
        except Exception as e:
            print(f"[process_frame] 弹窗显示 img 失败: {e}")

        if img is None:
            print("img is none")
            return

        if dets_obj is None:
            print("dets_obj is none")
            return

        print(f"img: {img}")
        # process_frame 需要 DataInterface 实例作为第一个参数
        masked_imgs = process_frame(self.di, img, dets_obj, T_map_to_body, self.count)
        self.count += 1
        # 弹窗显示 masked_img，阻塞直到用户按键关闭
        # try:
        #     import cv2
        #     cv2.imshow(f'masked_{type}', masked_img)
        #     cv2.waitKey(0)
        #     cv2.destroyWindow(f'masked_{type}')
        # except Exception as e:
        #     print(f"[process_frame] 弹窗显示 masked_img 失败: {e}")

        maps = []
        for img_m, type_m in masked_imgs:
            non_black = np.any(img_m != 0, axis=2)
            ys, xs = np.where(non_black)
            if len(xs) == 0:
                continue
            pts = list(zip(xs, ys))
            print(f"pts: {pts}")
            transformer = TfTransformer()
            originmap_pixels = []
            count_for = 0
            for u, v in pts:
                try:
                    print(f"u, v: {u, v}")
                    # print("=========into calc_point_on_origin_map=========")
                    res = transformer.calc_point_on_origin_map((u, v), T_map_to_body, count = count_for)
                    # print("========out calc_point_on_origin_map========")
                    print(f"res: {res}")
                    pixel = res.get('map_pixel')
                    if pixel is not None:
                        originmap_pixels.append(pixel)

                except Exception as e:
                    print("============= the fucking error is below ===========")
                    print(f"calc_point_on_origin_map error: {e}")
                    print("============= the fucking error is above ===========")
                    continue
                count_for += 1
            print(f"originmap_pixels: {originmap_pixels}")
            maps.append([originmap_pixels, type_m])

        any_drawn = False
        # print("=======================")
        print("开始画图")
        for _originmap_pixels, _ in maps:
            print(f"像素点数量{len(_originmap_pixels)}")
        for _originmap_pixels, type_m in maps:
            if not _originmap_pixels:
                continue
            # 按类别查找颜色（OCCUPANCY_COLOR_MAP 中是 RGB），传给 draw_point_on_map（它会把 RGB 转为 BGR）
            color_info = OCCUPANCY_COLOR_MAP.get(str(type_m)) or OCCUPANCY_COLOR_MAP.get(type_m)
            if color_info and 'rgb' in color_info:
                rgb = tuple(int(x) for x in color_info['rgb'])
            else:
                rgb = (0, 0, 255)  # 默认红色（RGB）
            # 使用持久化绘制函数，它会更新 self.drawer.map 并保存 debug 图像
            try:
                print("=============================")
                print("调用draw_point_on_map")
                print("此时的self.map:", self.drawer.map)
                print("=============================")
                drawn_img = self.drawer.draw_point_on_map(_originmap_pixels, origin_map_path=None, color=rgb, radius=2, out_path=None)
                any_drawn = True
                self.get_logger().info(f"[Pipeline] drew {len(_originmap_pixels)} pixels for type {type_m}")
                # 发布 debug 绘制结果到 ROS2 topic
                try:
                    if not hasattr(self, '_debug_img_pub'):
                        from sensor_msgs.msg import Image
                        self._debug_img_pub = self.create_publisher(Image, '/debug_drawn_img', 1)
                    bridge = self._bridge if hasattr(self, '_bridge') else CvBridge()
                    msg = bridge.cv2_to_imgmsg(drawn_img, encoding='bgr8')
                    self._debug_img_pub.publish(msg)
                    self.get_logger().info(f"[Pipeline] published debug drawn_img to /debug_drawn_img")
                except Exception as e:
                    self.get_logger().warn(f"[Pipeline] failed to publish debug drawn_img: {e}")
            except Exception as e:
                self.get_logger().warn(f"[Pipeline] draw_point_on_map failed for type {type_m}: {e}")
                # save the drawn origin-map image to debug dir with timestamp for inspection
                try:
                    from datetime import datetime
                    debug_dir = Path('/Pav-S_ws/src/costmap_process/new/debug')
                    debug_dir.mkdir(parents=True, exist_ok=True)
                    ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    draw_path = str(debug_dir / f'drawn_map_{type_m}_{ts}.png')
                    cv2.imwrite(draw_path, drawn_img)
                    self.get_logger().info(f"[Pipeline] saved drawn origin map to: {draw_path}")
                except Exception as _e:
                    self.get_logger().warn(f"[Pipeline] failed to save drawn origin map: {_e}")
        print("画完了")
        print("=============================")
        if any_drawn and getattr(self.di, '_node', None) is not None:
            try:
                self.get_logger().info(f"[Pipeline] attempting to publish occupancy from in-memory map (any_drawn={any_drawn}) using node={self.di._node}")
                # 使用 publish_occupancy_from_png 以利用 priority 覆盖策略并直接使用 self.map
                self.drawer.publish_occupancy_from_png(node=self.di._node, topic=None, show=False)
            except Exception as e:
                self.get_logger().warn(f"[Pipeline] publish_occupancy_from_png failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    pipeline = PipelineNode()
    pipeline.get_logger().info("ROS PipelineNode started.")
    rclpy.spin(pipeline)
    pipeline.destroy_node()
    rclpy.shutdown()




