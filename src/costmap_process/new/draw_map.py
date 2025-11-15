from .config import MAP_ORIGIN_PATH
from pathlib import Path
import cv2
import numpy as np


class DrawMap:
    def __init__(self, map_origin_path: str = MAP_ORIGIN_PATH):
        """管理 origin map 的加载、单点/多点绘制、显示与保存。"""
        self.map_origin_path = Path(map_origin_path) if map_origin_path else None
        self.map = None  # BGR numpy.ndarray
        self.img_size = None  # (w, h)
        # 延迟加载，不在构造时强制读取

    def load_map(self):
        """从 self.map_origin_path 读取图片到 self.map（若尚未加载）。"""
        if self.map is not None:
            return self.map
        if self.map_origin_path is None:
            raise ValueError('no map_origin_path provided')
        p = Path(self.map_origin_path)
        if not p.exists():
            raise FileNotFoundError(f'map origin not found: {p}')
        img = cv2.imread(str(p))
        if img is None:
            raise RuntimeError(f'failed to read map image: {p}')
        self.map = img.copy()
        h, w = img.shape[:2]
        self.img_size = (w, h)
        return self.map

    def draw_point(self, u: int, v: int, color=(0, 0, 255), radius: int = 1):
        """在内存地图 self.map 上绘制单个像素点（带黑色轮廓）。

        返回 True 表示绘制成功（点在图像范围内），否则 False。
        """
        if self.map is None:
            self.load_map()
        h, w = self.map.shape[:2]
        try:
            u_i = int(round(float(u)))
            v_i = int(round(float(v)))
        except Exception:
            return False
        if not (0 <= u_i < w and 0 <= v_i < h):
            return False
        outline_radius = max(1, int(radius) + 1)
        try:
            # cv2.circle(self.map, (u_i, v_i), outline_radius, (0, 0, 0), -1)
            cv2.circle(self.map, (u_i, v_i), int(radius), color, -1)
            return True
        except Exception:
            return False

    def draw_points(self, points, color=(0, 0, 255), radius: int = 1, out_path: str = None, show: bool = True):
        """在 self.map 上绘制多个点。

        points: 可迭代的 (u,v) 列表或 numpy array。
        返回 (img, drawn_points) 其中 img 为绘制后的图像副本，drawn_points 为实际绘制的像素列表。

        """
        if self.map is None:
            self.load_map()
        img = self.map.copy()
        h, w = img.shape[:2]

        pts_arr = np.asarray(points)
        if pts_arr.ndim == 1 and pts_arr.size == 2:
            pts_arr = pts_arr.reshape(1, 2)
        if pts_arr.ndim != 2 or pts_arr.shape[1] != 2:
            raise ValueError('points must be iterable of (u,v) pairs')

        # 打印范围供调试
        try:
            print(f'draw_map.draw_points: pts_arr min/max before swap: u [{pts_arr[:,0].min()},{pts_arr[:,0].max()}], v [{pts_arr[:,1].min()},{pts_arr[:,1].max()}]')
        except Exception:
            pass

        # 自动检测 (v,u) 颠倒
        first_col = pts_arr[:, 0]
        second_col = pts_arr[:, 1]
        count_first_oob = np.sum((first_col < 0) | (first_col >= w))
        count_second_in_h = np.sum((second_col >= 0) & (second_col < h))
        if count_first_oob > (len(pts_arr) * 0.6) and count_second_in_h > (len(pts_arr) * 0.6):
            pts_arr = pts_arr[:, ::-1]
            print('draw_map.draw_points: swapped columns assuming (v,u) was passed')

        drawn = []
        for pt in pts_arr:
            try:
                u_i = int(round(float(pt[0]))); v_i = int(round(float(pt[1])))
            except Exception as e:
                print(f"[DrawMap][draw_points] line93 : {e}")
                continue
            if 0 <= u_i < w and 0 <= v_i < h:
                outline_radius = max(1, int(radius) + 1)
                try:
                    # cv2.circle(img, (u_i, v_i), outline_radius, (0, 0, 0), -1)
                    cv2.circle(img, (u_i, v_i), int(radius), color, -1)
                except Exception as e:
                    print(f"[DrawMap][draw_points] failed to draw point ({u_i}, {v_i}): {e}")
                    continue
                drawn.append((u_i, v_i))

        if drawn:
            arr = np.array(drawn, dtype=int)
            print(f'draw_map.draw_points: drew {len(drawn)} points. u range [{arr[:,0].min()},{arr[:,0].max()}], v range [{arr[:,1].min()},{arr[:,1].max()}]')
        else:
            print('draw_map.draw_points: no points drawn (all out of image bounds?)')

        # 保存到文件（若提供）
        if out_path:
            try:
                cv2.imwrite(str(out_path), img)
            except Exception:
                pass

        # 若需显示，则弹窗并等待按键
        if show:
            # cv2.imshow('points_on_origin_map', img)
            # cv2.waitKey(0)
            # cv2.destroyWindow('points_on_origin_map')
            pass
        
        # 更新内存地图为绘制后的结果（可选，保留原图为 self.map）
        self.map = img
        return img, drawn

    def save(self, out_path: str):
        """把当前 self.map 保存到文件。"""
        if self.map is None:
            self.load_map()
        try:
            cv2.imwrite(str(out_path), self.map)
            return True
        except Exception:
            return False

    def draw_point_on_map(self, points, origin_map_path: str = None, color=(0,0,255), radius=1, out_path: str = None):
        """在 origin_map 上绘制像素点并持久化到 self.map（除非被覆盖，否则不会被消除）。

        参数:
            points: 可迭代的 (u,v) 列表或 numpy array（像素坐标）
            origin_map_path: 若提供则加载此图片作为底图，否则使用 self.map 或 self.map_origin_path
            color: RGB 三元组（注意 OpenCV 为 BGR）
            radius: 点半径
            out_path: 若提供则把绘制结果保存到该路径

        返回: 绘制后的图像 numpy.ndarray
        """
        # 解析并加载底图
        img = None
        if origin_map_path:
            p = Path(origin_map_path)
            if p.exists():
                img = cv2.imread(str(p))
        if img is None and self.map is not None:
            img = self.map.copy()
        if img is None and self.map_origin_path is not None and Path(self.map_origin_path).exists():
            img = cv2.imread(str(self.map_origin_path))
        if img is None:
            raise ValueError('no img to draw (provide origin_map_path or ensure self.map/self.map_origin_path exists)')

        h, w = img.shape[:2]
        # 规范 points
        pts_arr = np.asarray(points)
        if pts_arr.ndim == 1 and pts_arr.size == 2:
            pts_arr = pts_arr.reshape(1, 2)
        if pts_arr.ndim != 2 or pts_arr.shape[1] != 2:
            raise ValueError('points must be iterable of (u,v) pairs')

        # 自动检测是否传入 (v,u)
        try:
            print(f'draw_point_on_map: loaded image shape=(h={h}, w={w})')
            print(f'draw_point_on_map: transformer.img_size={self.img_size}')
        except Exception:
            pass
        first_col = pts_arr[:, 0]
        second_col = pts_arr[:, 1]
        count_first_oob = np.sum((first_col < 0) | (first_col >= w))
        count_second_in_h = np.sum((second_col >= 0) & (second_col < h))
        if count_first_oob > (len(pts_arr) * 0.6) and count_second_in_h > (len(pts_arr) * 0.6):
            pts_arr = pts_arr[:, ::-1]
            print('draw_point_on_map: swapped columns assuming (v,u) was passed')

        drawn = []
        for pt in pts_arr:
            try:
                u = int(round(float(pt[0]))); v = int(round(float(pt[1])))
            except Exception:
                continue
            if 0 <= u < w and 0 <= v < h:
                outline_radius = max(1, int(radius) + 1)
                try:
                    # cv2.circle(img, (u, v), outline_radius, (0, 0, 0), -1)
                    # color param is RGB in user's snippet; convert to BGR for OpenCV
                    bgr_color = (int(color[2]), int(color[1]), int(color[0])) if len(color) == 3 else color
                    cv2.circle(img, (u, v), int(radius), bgr_color, -1)
                except Exception:
                    continue
                drawn.append((u, v))

        if drawn:
            arr = np.array(drawn, dtype=int)
            print(f'draw_point_on_map: drew {len(drawn)} points. u range [{arr[:,0].min()},{arr[:,0].max()}], v range [{arr[:,1].min()},{arr[:,1].max()}]')
            print('example points:', drawn[:5])
        else:
            print('draw_point_on_map: no points drawn (all out of image bounds?)')

        # 持久化：更新 self.map 并可选择写文件
        self.map = img
        if out_path:
            try:
                cv2.imwrite(str(out_path), img)
            except Exception:
                pass
        # 始终把每次绘制的结果保存到 debug 目录，便于离线查看
        try:
            from datetime import datetime
            debug_dir = Path('/Pav-S_ws/src/costmap_process/new/debug')
            debug_dir.mkdir(parents=True, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            debug_auto_path = str(debug_dir / f'draw_point_on_map_{ts}.png')
            cv2.imwrite(debug_auto_path, img)
            print(f'[DrawMap] draw_point_on_map saved debug image: {debug_auto_path}')
        except Exception as _e:
            print(f'[DrawMap] failed to save debug image: {_e}')
        return img

    def build_costmap_arrays(self, color_map: dict = None, color_threshold: int = None):
        """把当前地图转换为 nav2 costmap（0-254-255）与 OccupancyGrid（-1/0/100）二维数组。

        返回包含 costmap/occupancy 数组及宽高、分类统计的字典。
        """
        if self.map is None:
            self.load_map()

        if self.map is None:
            raise RuntimeError('build_costmap_arrays: no map image available')

        img = self.map
        h, w = img.shape[:2]

        from .config import (
            OCCUPANCY_COLOR_MAP,
            OCCUPANCY_COLOR_THRESHOLD,
        )

        cmap = color_map if color_map is not None else OCCUPANCY_COLOR_MAP
        thr = color_threshold if color_threshold is not None else OCCUPANCY_COLOR_THRESHOLD
        if thr is None:
            thr = 12

        costmap = np.full((h, w), 255, dtype=np.uint8)
        occupancy = np.full((h, w), -1, dtype=np.int16)

        img_bgr = img.astype(np.int16)
        items = []
        for key, info in cmap.items():
            priority = info.get('priority', 1000)
            items.append((priority, key, info))
        items.sort(reverse=True)

        summary = []
        thr2 = thr * thr

        for priority, key, info in items:
            rgb = info.get('rgb')
            if rgb is None:
                summary.append({'key': key, 'matched': 0})
                continue

            target = np.array([int(rgb[2]), int(rgb[1]), int(rgb[0])], dtype=np.int16)
            diff = img_bgr - target.reshape((1, 1, 3))
            dist2 = np.sum(diff * diff, axis=2)
            mask = dist2 <= thr2
            matched = int(np.count_nonzero(mask))
            if matched == 0:
                summary.append({'key': key, 'matched': 0})
                continue

            occ_val = int(info.get('occ', 100))
            occ_val = int(max(-1, min(100, occ_val)))
            if occ_val < 0:
                cost_val = 255
            elif occ_val >= 100:
                cost_val = 254
            else:
                cost_val = int(round((occ_val / 100.0) * 254))
            cost_val = int(max(0, min(254, cost_val)))

            occupancy[mask] = occ_val
            costmap[mask] = cost_val
            summary.append({'key': key, 'matched': matched, 'occ': occ_val, 'cost': cost_val})

        return {
            'costmap': costmap,
            'occupancy': occupancy,
            'width': w,
            'height': h,
            'summary': summary,
        }

    def build_costmap_messages(self, *, stamp, map_frame: str = 'map', layer_name: str = 'perception_layer',
                               resolution: float = None, color_map: dict = None, color_threshold: int = None,
                               mirror_y: bool = True, map_load_time=None, origin_xy=None, origin_yaw: float = 0.0):
        """基于当前地图生成 nav2_msgs/Costmap 与 nav_msgs/OccupancyGrid 消息。"""
        arrays = self.build_costmap_arrays(color_map=color_map, color_threshold=color_threshold)

        costmap_array = arrays['costmap']
        occupancy_array = arrays['occupancy']
        h = arrays['height']
        w = arrays['width']

        try:
            from geometry_msgs.msg import Pose, Point, Quaternion
            from nav2_msgs.msg import Costmap, CostmapMetaData
            from nav_msgs.msg import OccupancyGrid, MapMetaData
        except ImportError as exc:
            raise RuntimeError(f'ROS2 message imports failed: {exc}') from exc

        from .config import (
            MAP_ROI_MIN,
            MAP_ROI_MAX,
            DEFAULT_OCCUPANCY_RESOLUTION,
        )

        if resolution is None or resolution <= 0.0:
            if DEFAULT_OCCUPANCY_RESOLUTION is not None:
                resolution = float(DEFAULT_OCCUPANCY_RESOLUTION)
            else:
                x_min, y_min = MAP_ROI_MIN
                x_max, y_max = MAP_ROI_MAX
                width = float(max(1, w))
                resolution = (x_max - x_min) / width

        if origin_xy is None:
            origin_xy = MAP_ROI_MIN

        import math

        pose = Pose()
        pose.position = Point(x=float(origin_xy[0]), y=float(origin_xy[1]), z=0.0)
        # 仅支持平面 yaw，绕 Z 轴旋转
        half_yaw = float(origin_yaw) * 0.5
        pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=float(math.sin(half_yaw)),
            w=float(math.cos(half_yaw)),
        )

        if not mirror_y:
            costmap_use = np.flipud(costmap_array)
            occupancy_use = np.flipud(occupancy_array)
        else:
            costmap_use = costmap_array
            occupancy_use = occupancy_array

        costmap_msg = Costmap()
        costmap_msg.header.frame_id = map_frame
        costmap_msg.header.stamp = stamp

        metadata = CostmapMetaData()
        metadata.resolution = float(resolution)
        metadata.size_x = int(w)
        metadata.size_y = int(h)
        metadata.layer = layer_name
        metadata.origin = pose
        metadata.map_load_time = map_load_time if map_load_time is not None else stamp
        metadata.update_time = stamp
        costmap_msg.metadata = metadata
        costmap_msg.data = costmap_use.reshape(-1).astype(np.uint8).tolist()

        occ_msg = OccupancyGrid()
        occ_msg.header.frame_id = map_frame
        occ_msg.header.stamp = stamp

        occ_info = MapMetaData()
        occ_info.resolution = float(resolution)
        occ_info.width = int(w)
        occ_info.height = int(h)
        occ_info.origin = pose
        occ_msg.info = occ_info

        occ_flat = occupancy_use.reshape(-1)
        occ_msg.data = [int(max(-1, min(100, int(v)))) for v in occ_flat]

        return costmap_msg, occ_msg, arrays['summary']

    def publish_occupancy_from_png(self, png_path: str = None, node=None, publisher=None, topic: str = None,
                                   frame_id: str = 'map', resolution: float = None,
                                   color_map: dict = None, color_threshold: int = None,
                                   show: bool = False, mirror_y: bool = True):
        """从给定 PNG（或 self.map）生成并发布 OccupancyGrid。

        行为说明：
        - 优先使用 png_path 指定的图片；否则使用 self.map（若为空则尝试 load_map）。
        - 颜色到 occ 的映射使用 color_map（或 config.OCCUPANCY_COLOR_MAP）。
        - 覆盖策略按 priority 控制：优先级数值越小表示越高优先级，会在最后覆盖低优先级的像素，
          从而满足“画上去后不会被消除，除非被更高优先级覆盖”的需求。
        """
        # 读取图片：优先使用内存中的 self.map（避免读盘），只有在 self.map 为空时才尝试加载 png_path 或 map_origin
        img = None
        if self.map is not None:
            img = self.map.copy()
        else:
            if png_path:
                p = Path(png_path)
                if p.exists():
                    img = cv2.imread(str(p))
            if img is None:
                # 最后退回到 map_origin_path
                if self.map is None:
                    self.load_map()
                img = self.map.copy()
        if img is None:
            raise RuntimeError('no image available to build occupancy grid')

        h, w = img.shape[:2]

        # color map and threshold
        from .config import OCCUPANCY_TOPIC

        if node is None:
            raise RuntimeError('publish_occupancy_from_png 需要传入 node 或 publisher')

        stamp = node.get_clock().now().to_msg()
        costmap_msg, occ_msg, summary = self.build_costmap_messages(
            stamp=stamp,
            map_frame=frame_id,
            layer_name='legacy_layer',
            resolution=resolution,
            color_map=color_map,
            color_threshold=color_threshold,
            mirror_y=mirror_y,
            map_load_time=None,
        )

        pub_to_use = publisher
        if pub_to_use is None:
            topic_name = topic if topic is not None else OCCUPANCY_TOPIC
            pub_to_use = node.create_publisher(type(occ_msg), topic_name, 1)

        occ_only = occ_msg
        pub_to_use.publish(occ_only)
        print(f"[DrawMap] publish_from_png: published topic={topic if topic else OCCUPANCY_TOPIC} width={w} height={h}")
        for item in summary:
            if item.get('matched', 0) > 0:
                print(f"    category={item['key']} matched={item['matched']} occ={item.get('occ')} cost={item.get('cost')}")

        # 可视化并保存 debug 图像（与原 publish 函数相同方式）
        vis = np.zeros((h, w, 3), dtype=np.uint8)
        value2color = {}
        for item in summary:
            if 'occ' in item and 'cost' in item:
                key = item.get('occ', -1)
                matched_key = item['key']
                rgb = None
                if color_map and matched_key in color_map:
                    rgb = color_map[matched_key].get('rgb')
                else:
                    from .config import OCCUPANCY_COLOR_MAP
                    if matched_key in OCCUPANCY_COLOR_MAP:
                        rgb = OCCUPANCY_COLOR_MAP[matched_key].get('rgb')
                if rgb:
                    value2color[key] = (int(rgb[2]), int(rgb[1]), int(rgb[0]))
        occ_matrix = np.array(occ_msg.data, dtype=np.int16).reshape(h, w)
        for y in range(h):
            for x in range(w):
                v = int(occ_matrix[y, x])
                if v == -1:
                    vis[y, x] = (127, 127, 127)
                elif v in value2color:
                    vis[y, x] = value2color[v]
                else:
                    vis[y, x] = (200, 200, 200)
        try:
            from datetime import datetime
            debug_dir = Path('/Pav-S_ws/src/costmap_process/new/debug')
            debug_dir.mkdir(parents=True, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            debug_path = str(debug_dir / f'occupancy_from_png_{ts}.png')
            cv2.imwrite(debug_path, vis)
            print(f'[DrawMap] occupancy_from_png visual saved: {debug_path}')
        except Exception as e:
            print(f'[DrawMap] save visual failed: {e}')

        if show:
            cv2.imshow('occupancy_from_png', vis)
            cv2.waitKey(1)

        return occ_only




