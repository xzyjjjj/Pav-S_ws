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

    def publish_occupancy_from_png(self, png_path: str = None, node=None, publisher=None, topic: str = None,
                                   frame_id: str = 'map', resolution: float = None,
                                   color_map: dict = None, color_threshold: int = None,
                                   show: bool = False):
        """从给定 PNG（或 self.map）生成并发布 OccupancyGrid。

        行为说明：
        - 优先使用 png_path 指定的图片；否则使用 self.map（若为空则尝试 load_map）。
        - 颜色到 occ 的映射使用 color_map（或 config.OCCUPANCY_COLOR_MAP）。
        - 覆盖策略按 priority 控制：优先级数值越小表示越高优先级，会在最后覆盖低优先级的像素，
          从而满足“画上去后不会被消除，除非被更高优先级覆盖”的需求。
        """
        # 延迟导入 ROS2 types
        try:
            from nav_msgs.msg import OccupancyGrid, MapMetaData
            from std_msgs.msg import Header
            from geometry_msgs.msg import Pose, Point, Quaternion
            import rclpy
        except Exception as e:
            raise RuntimeError(f'ROS2 imports failed: {e}')

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
        from .config import MAP_ROI_MIN, MAP_ROI_MAX, OCCUPANCY_COLOR_MAP, OCCUPANCY_COLOR_THRESHOLD, OCCUPANCY_TOPIC, DEFAULT_OCCUPANCY_RESOLUTION
        cmap = color_map if color_map is not None else OCCUPANCY_COLOR_MAP
        thr = color_threshold if color_threshold is not None else OCCUPANCY_COLOR_THRESHOLD
        if thr is None:
            thr = 30

        if resolution is None:
            if DEFAULT_OCCUPANCY_RESOLUTION is not None:
                resolution = DEFAULT_OCCUPANCY_RESOLUTION
            else:
                x_min, y_min = MAP_ROI_MIN
                x_max, y_max = MAP_ROI_MAX
                resolution = float(x_max - x_min) / float(w) if w > 0 else 0.01

        grid = np.full((h, w), -1, dtype=np.int8)

        img_bgr = img.astype(np.int16)
        # build items list (pr, key, info)
        items = []
        for k, v in cmap.items():
            pr = v.get('priority', 1000)
            items.append((pr, k, v))
        # apply lower-priority first (higher numeric pr first), so small pr (high priority) applied last to overwrite
        items.sort(reverse=True)

        for pr, key, info in items:
            rgb = info.get('rgb')
            occ_val = int(info.get('occ', 100))
            if rgb is None:
                continue
            target = np.array([int(rgb[2]), int(rgb[1]), int(rgb[0])], dtype=np.int16)
            diff = img_bgr - target.reshape((1, 1, 3))
            dist2 = np.sum(diff * diff, axis=2)
            mask = dist2 <= (thr * thr)
            try:
                matched = int(np.sum(mask))
            except Exception:
                matched = 0
            print(f"[DrawMap] publish_from_png checking category='{key}' rgb={rgb} occ={occ_val} matched_pixels={matched}")
            if not np.any(mask):
                continue
            # 按 priority 覆盖（直接赋值），以便高优先级最后覆盖低优先级
            grid[mask] = occ_val

        # 构造 OccupancyGrid message (与 publish_occupancy_map 保持一致)
        msg = OccupancyGrid()
        hdr = Header()
        hdr.stamp = node.get_clock().now().to_msg() if node is not None else rclpy.time.Time().to_msg()
        hdr.frame_id = frame_id
        msg.header = hdr

        info = MapMetaData()
        info.resolution = float(resolution)
        info.width = int(w)
        info.height = int(h)
        pose = Pose()
        pose.position = Point(x=float(MAP_ROI_MIN[0]), y=float(MAP_ROI_MIN[1]), z=0.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        info.origin = pose
        msg.info = info

        grid_flipped = np.flipud(grid)
        flat = grid_flipped.reshape(-1).tolist()
        flat = [int(max(-1, min(100, int(x)))) for x in flat]
        msg.data = flat

        pub_to_use = publisher
        if pub_to_use is None:
            if node is None:
                raise RuntimeError('either publisher or node must be provided to publish_occupancy_map')
            topic_name = topic if topic is not None else OCCUPANCY_TOPIC
            pub_to_use = node.create_publisher(OccupancyGrid, topic_name, 1)

        pub_to_use.publish(msg)
        print(f"[DrawMap] publish_from_png: published topic={topic if topic else OCCUPANCY_TOPIC} width={w} height={h}")

        # 可视化并保存 debug 图像（与原 publish 函数相同方式）
        vis = np.zeros((h, w, 3), dtype=np.uint8)
        value2color = {}
        for k, vinfo in cmap.items():
            occ_val = int(vinfo.get('occ', 100))
            rgb = vinfo.get('rgb')
            if rgb:
                value2color[occ_val] = (int(rgb[2]), int(rgb[1]), int(rgb[0]))
        for y in range(h):
            for x in range(w):
                v = int(grid[y, x])
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

        return msg

    def publish_occupancy_map(self, node=None, publisher=None, topic: str = None,
                              frame_id: str = 'map', resolution: float = None,
                              color_map: dict = None, color_threshold: int = None,
                              show: bool = False):
        """把当前内存地图转换为 nav_msgs/OccupancyGrid 并通过给定的 publisher 或 node 发布。

        参数:
            node: rclpy.node.Node，如果提供且 publisher 为 None，会使用 node.create_publisher 创建 publisher
            publisher: 已创建的 rclpy publisher（nav_msgs.msg.OccupancyGrid）对象
            topic: 如果需要由本函数创建 publisher，可以传入 topic 名称（覆盖 config 中的默认）
            frame_id: OccupancyGrid.header.frame_id，默认 'map'
            resolution: 地图分辨率（米/像素）。若为 None，则尝试从 config 的 MAP_ROI 与图片尺寸计算
            color_map: 可选的颜色映射表（同 config.OCCUPANCY_COLOR_MAP 结构）
            color_threshold: RGB 距离阈值，用于颜色匹配
            show: 是否在本地弹窗显示生成的 occupancy 可视化（仅图片，不影响发布）

        返回: 发布的 OccupancyGrid 消息对象（若成功），否则抛出异常。
        """
        # 延迟导入 ROS2 以避免在非 ROS 环境导入错误
        try:
            # import here to make module import safe outside ROS runtime
            from nav_msgs.msg import OccupancyGrid, MapMetaData
            from std_msgs.msg import Header
            from geometry_msgs.msg import Pose, Point, Quaternion
            import rclpy
        except Exception as e:
            raise RuntimeError(f'ROS2 imports failed: {e}')

        # 加载 map
        if self.map is None:
            self.load_map()
        img = self.map.copy()
        h, w = img.shape[:2]

        # color map and threshold
        from .config import MAP_ROI_MIN, MAP_ROI_MAX, OCCUPANCY_COLOR_MAP, OCCUPANCY_COLOR_THRESHOLD, OCCUPANCY_TOPIC, DEFAULT_OCCUPANCY_RESOLUTION
        cmap = color_map if color_map is not None else OCCUPANCY_COLOR_MAP
        thr = color_threshold if color_threshold is not None else OCCUPANCY_COLOR_THRESHOLD
        # 防御性处理：若配置中没有阈值则使用一个合理的默认值（RGB 距离阈值）
        if thr is None:
            thr = 30

        # 计算分辨率（米/像素），优先参数传入，其次使用 config 中的默认，否则从 MAP_ROI 计算
        if resolution is None:
            if DEFAULT_OCCUPANCY_RESOLUTION is not None:
                resolution = DEFAULT_OCCUPANCY_RESOLUTION
            else:
                x_min, y_min = MAP_ROI_MIN
                x_max, y_max = MAP_ROI_MAX
                # horizontal resolution (meters per pixel)
                resolution = float(x_max - x_min) / float(w) if w > 0 else 0.01

        # 准备输出数组，默认 -1 (unknown)
        grid = np.full((h, w), -1, dtype=np.int8)

        # 颜色匹配：输入图像为 BGR，cmap 存的是 RGB -> convert
        img_bgr = img.astype(np.int16)
        # sort categories by priority ascending so later (higher priority value) can overwrite if desired
        # but we want highest priority (small numeric value) to override, so sort by priority descending
        items = []
        for k, v in cmap.items():
            pr = v.get('priority', 1000)
            items.append((pr, k, v))
        # sort so lower pr (higher priority) applied last to overwrite lower priority if overlapping
        items.sort(reverse=True)

        for pr, key, info in items:
            rgb = info.get('rgb')
            occ_val = int(info.get('occ', 100))
            if rgb is None:
                continue
            # convert RGB->BGR for comparison
            target = np.array([int(rgb[2]), int(rgb[1]), int(rgb[0])], dtype=np.int16)
            # compute squared distance to be faster
            diff = img_bgr - target.reshape((1, 1, 3))
            dist2 = np.sum(diff * diff, axis=2)
            # thr 为 RGB 距离阈值，比较平方距离以避免开根运算
            mask = dist2 <= (thr * thr)
            # 调试：打印每个类别被匹配的像素数量
            try:
                matched = int(np.sum(mask))
            except Exception:
                matched = 0
            print(f"[DrawMap] checking category='{key}' rgb={rgb} occ={occ_val} matched_pixels={matched}")
            if not np.any(mask):
                continue
            # for all masked pixels, set grid value to max(existing, occ_val) (方案 A 要求 max occupancy)
            # 注意：grid 的索引 (iy, ix) 对应图像 (v,u)；image origin is top-left, occupancy grid origin we'll set to lower-left later
            # Here we just set grid[v, u]
            # existing -1 means unknown; set to occ_val
            existing = grid[mask]
            # if existing == -1 -> replace; else take max
            to_set = np.where(existing == -1, occ_val, np.maximum(existing, occ_val))
            grid[mask] = to_set

        # 输出准备发布的像素点范围
        valid_pts = np.argwhere(grid != -1)
        if valid_pts.size > 0:
            u_min, v_min = valid_pts[:,1].min(), valid_pts[:,0].min()
            u_max, v_max = valid_pts[:,1].max(), valid_pts[:,0].max()
            print(f"[DrawMap] occupancy 发布像素点范围: u [{u_min},{u_max}], v [{v_min},{v_max}], 总数: {valid_pts.shape[0]}")
        else:
            print("[DrawMap] occupancy 发布像素点范围: 无有效点")

        # ----------------- 调试输出: 打印当前 map 的颜色->occ 映射与统计信息 -----------------
        try:
            # 构建 occ -> (category names list, rgb, bgr) 映射以便打印
            occ_to_names = {}
            occ_to_rgb = {}
            occ_to_bgr = {}
            for name, info in cmap.items():
                occ_val = int(info.get('occ', 100))
                rgb = info.get('rgb')
                occ_to_names.setdefault(occ_val, []).append(name)
                if rgb is not None:
                    occ_to_rgb[occ_val] = tuple(int(x) for x in rgb)
                    occ_to_bgr[occ_val] = (int(rgb[2]), int(rgb[1]), int(rgb[0]))

            print('[DrawMap][DEBUG] value2color mapping and categories:')
            for occ_val in sorted(occ_to_names.keys(), reverse=True):
                names = ','.join(occ_to_names.get(occ_val, []))
                rgb = occ_to_rgb.get(occ_val, None)
                bgr = occ_to_bgr.get(occ_val, None)
                print(f"  occ={occ_val}\tcategories={names}\tRGB={rgb}\tBGR={bgr}")

            # 统计 grid 中每个 occ 的像素数量
            vals, counts = np.unique(grid, return_counts=True)
            total_pixels = int(grid.size)
            unk_count = 0
            print('[DrawMap][DEBUG] occupancy counts (value -> count, percent):')
            for v, c in zip(vals, counts):
                pct = 100.0 * int(c) / total_pixels
                if int(v) == -1:
                    unk_count = int(c)
                    print(f"  -1 (unknown) -> {int(c)} pixels ({pct:.2f}%)")
                else:
                    print(f"  {int(v)} -> {int(c)} pixels ({pct:.2f}%)")

            if total_pixels > 0:
                print(f"[DrawMap][DEBUG] known pixels: {total_pixels - unk_count}, unknown: {unk_count}")

            # 打印若干示例像素 (最多10个)，显示其坐标、图像的 BGR 值以及 grid 中的 occ
            if valid_pts.size > 0:
                # valid_pts 的每行是 (v,u)
                sample_n = min(10, valid_pts.shape[0])
                # 随机选择 sample_n 个索引
                idxs = np.random.choice(valid_pts.shape[0], size=sample_n, replace=False)
                print('[DrawMap][DEBUG] sample pixels (u,v) -> BGR -> occ -> category_names')
                for ii in idxs:
                    vy, ux = int(valid_pts[ii,0]), int(valid_pts[ii,1])
                    bgr_px = tuple(int(x) for x in img[vy, ux])
                    occ_px = int(grid[vy, ux])
                    names = occ_to_names.get(occ_px, ['<unknown>'])
                    print(f"  ({ux},{vy}) -> BGR={bgr_px} -> occ={occ_px} -> {','.join(names)}")
        except Exception as _e:
            print(f"[DrawMap][DEBUG] failed to print debug info: {_e}")
        # ----------------- 调试输出结束 -----------------

        # 构造 OccupancyGrid message
        msg = OccupancyGrid()
        # header
        hdr = Header()
        hdr.stamp = node.get_clock().now().to_msg() if node is not None else rclpy.time.Time().to_msg()
        hdr.frame_id = frame_id
        msg.header = hdr

        # info
        info = MapMetaData()
        info.resolution = float(resolution)
        info.width = int(w)
        info.height = int(h)
        # origin pose: put at MAP_ROI_MIN (lower-left), orientation identity
        pose = Pose()
        pose.position = Point(x=float(MAP_ROI_MIN[0]), y=float(MAP_ROI_MIN[1]), z=0.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        info.origin = pose
        msg.info = info

        # OccupancyGrid.data is row-major starting at origin (lower-left). Our grid currently indexed [v,u] with v top-left.
        # Need to flip vertical axis and then flatten with row-major where row = y from 0 (lower) to h-1 (upper)
        grid_flipped = np.flipud(grid)
        flat = grid_flipped.reshape(-1).tolist()
        # ensure int values in [-1,100]
        flat = [int(max(-1, min(100, int(x)))) for x in flat]
        msg.data = flat

        # 发布
        pub_to_use = publisher
        if pub_to_use is None:
            if node is None:
                raise RuntimeError('either publisher or node must be provided to publish_occupancy_map')
            topic_name = topic if topic is not None else OCCUPANCY_TOPIC
            pub_to_use = node.create_publisher(OccupancyGrid, topic_name, 1)

        pub_to_use.publish(msg)

        print(f"[DrawMap] occupancy 发布: topic={topic if topic else OCCUPANCY_TOPIC}, frame_id={frame_id}, resolution={resolution}, width={w}, height={h}")
        print(f"[DrawMap] occupancy 发布: publisher={pub_to_use}")
        print(f"[DrawMap] occupancy 发布: msg.data 长度={len(msg.data)}")
        print("[DrawMap] occupancy 发布完成！")

        # 保存 occupancy 彩色可视化图到 debug 目录，便于调试
        vis = np.zeros((h, w, 3), dtype=np.uint8)
        # 构建类别值到颜色的映射表（BGR）
        value2color = {}
        for k, vinfo in cmap.items():
            occ_val = int(vinfo.get('occ', 100))
            rgb = vinfo.get('rgb')
            if rgb:
                value2color[occ_val] = (int(rgb[2]), int(rgb[1]), int(rgb[0]))
        for y in range(h):
            for x in range(w):
                v = int(grid[y, x])
                if v == -1:
                    vis[y, x] = (127, 127, 127)
                elif v in value2color:
                    vis[y, x] = value2color[v]
                else:
                    # 未知类别用灰色
                    vis[y, x] = (200, 200, 200)
        # 构建带时间戳的文件名，避免每次覆盖并便于回溯
        try:
            from datetime import datetime
            debug_dir = Path('/Pav-S_ws/src/costmap_process/new/debug')
            if not debug_dir.exists():
                try:
                    debug_dir.mkdir(parents=True, exist_ok=True)
                except Exception:
                    pass
            ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            debug_path = str(debug_dir / f'occupancy_debug_{ts}.png')
            cv2.imwrite(debug_path, vis)
            print(f'[DrawMap] occupancy 彩色可视化图已保存到: {debug_path}')
        except Exception as e:
            print(f'[DrawMap] occupancy 彩色可视化图保存失败: {e}')
        # 追加发布日志，方便确认发布次数与时间
        try:
            log_path = Path('/Pav-S_ws/src/costmap_process/new/debug/publish_log.txt')
            with log_path.open('a', encoding='utf-8') as fh:
                from datetime import datetime as _dt
                now = _dt.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                pts_count = int(valid_pts.shape[0]) if 'valid_pts' in locals() else 0
                topic_name = topic if topic is not None else OCCUPANCY_TOPIC if 'OCCUPANCY_TOPIC' in locals() else 'unknown'
                fh.write(f"{now}\tpublished\ttopic={topic_name}\tpoints={pts_count}\n")
        except Exception:
            pass
        if show:
            cv2.imshow('occupancy_vis', vis)
            cv2.waitKey(1)

        return msg

    
