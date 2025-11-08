from pathlib import Path
import cv2
import numpy as np
from numpy.linalg import inv
from .cal_xyz import calculate_xyz
from .config import *
class TfTransformer:
    def __init__(self, map_img_path: str = MAP_ORIGIN_PATH, map_range=((0,0),(5,3))):
        """初始化转换器。

        参数:
            map_img_path: origin map 图片路径（用于获取图片宽高以便像素投影）
            map_range: ((x_min,y_min),(x_max,y_max)) 地图在世界坐标系中的范围（米）
        """
        self.map_img_path = Path(map_img_path)
        self.map_range = map_range
        if self.map_img_path.exists():
            img = cv2.imread(str(self.map_img_path))
            if img is None:
                # 无法读取图片，延迟读取时再报错
                self.img_size = None
            else:
                h, w = img.shape[:2]
                self.img_size = (w, h)
        else:
            self.img_size = None
        # 缓存已加载的 tf，key 为路径字符串，value 为 4x4 矩阵
        self._tf_cache = {}
    
    def pixel_to_camera(self, img: np.ndarray, bbox_point: str):
        """从图片数组和 bbox 文件计算检测框中心点在相机坐标系下的 3D 点。

        参数:
            img: 图片数组（numpy.ndarray，BGR格式）
            bbox_txt: bbox 文本路径（yolo 格式：0 x_center y_center w h 归一化）
            camera_height: 地面相对于相机的高度，用于 cal_xyz 返回的 Y 分量
        返回:
        """
        if img is None:
            raise ValueError("img 参数不能为空，需为 numpy.ndarray")

        # 解析 bbox_point：支持 (u,v) 单点，或 4 个角点列表/tuple（取中心）
        u = v = None
        try:
            if bbox_point is None:
                raise ValueError('bbox_point is None')
            # 如果传入的是单个点 (u,v)
            if isinstance(bbox_point, (list, tuple)) and len(bbox_point) == 2 and isinstance(bbox_point[0], (int, float)):
                u = int(round(bbox_point[0])); v = int(round(bbox_point[1]))
            # 如果是 4 个角点
            elif isinstance(bbox_point, (list, tuple)) and len(bbox_point) >= 4:
                xs = [p[0] for p in bbox_point]
                ys = [p[1] for p in bbox_point]
                u = int(round(sum(xs) / len(xs)))
                v = int(round(sum(ys) / len(ys)))
            else:
                raise ValueError('unsupported bbox_point format')
        except Exception as e:
            raise ValueError(f'pixel_to_camera: cannot parse bbox_point: {e}')

        # 统一以实例方式调用 calculate_xyz（cal_xyz 中实现为实例方法）
        calc_inst = calculate_xyz()
        x, y, z = calc_inst.calc_xyz(u, v)
        # print("===================")
        # print(f"cam (x, y, z) = {x, y, z}")
        camera_pt = np.array([x, y, z], dtype=np.float64)
        return camera_pt

    def get_T_MAP_TO_BODY(self, tf_path: str = '/Users/xuzhenyu/Desktop/reutrn_stage/Pav-S_ws/src/costmap_process/scripts/collected_data/collect1/000007_tf.json'):
        """返回 map 到 body 的变换矩阵。"""
        # 支持多种输入：
        # - tf_path 为字符串路径 -> 读取 json 文件
        # - tf_path 为 numpy.ndarray (4x4) -> 直接返回
        # - tf_path 为 dict/obj 包含 translation, rotation -> 解析
        try:
            # numpy matrix passed directly
            if isinstance(tf_path, np.ndarray) and tf_path.shape == (4, 4):
                return tf_path
        except Exception:
            pass

        # dict-like
        if isinstance(tf_path, dict):
            obj = tf_path
            T_MAP_TO_BODY = np.eye(4, dtype=np.float64)
            try:
                tr = obj.get('translation', {})
                rot = obj.get('rotation', {})
                tx = float(tr.get('x', 0.0))
                ty = float(tr.get('y', 0.0))
                tz = float(tr.get('z', 0.0))
                qx = float(rot.get('x', 0.0))
                qy = float(rot.get('y', 0.0))
                qz = float(rot.get('z', 0.0))
                qw = float(rot.get('w', 1.0))
                R = np.zeros((3, 3), dtype=np.float64)
                R[0, 0] = 1 - 2 * (qy * qy + qz * qz)
                R[0, 1] = 2 * (qx * qy - qz * qw)
                R[0, 2] = 2 * (qx * qz + qy * qw)
                R[1, 0] = 2 * (qx * qy + qz * qw)
                R[1, 1] = 1 - 2 * (qx * qx + qz * qz)
                R[1, 2] = 2 * (qy * qz - qx * qw)
                R[2, 0] = 2 * (qx * qz - qy * qw)
                R[2, 1] = 2 * (qy * qz + qx * qw)
                R[2, 2] = 1 - 2 * (qx * qx + qy * qy)
                T_MAP_TO_BODY[:3, :3] = R
                T_MAP_TO_BODY[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
            except Exception:
                T_MAP_TO_BODY = np.eye(4, dtype=np.float64)
            return T_MAP_TO_BODY

        # otherwise assume path string
        from pathlib import Path
        import json

        p = Path(tf_path)
        key = str(p)
        # 如果已经缓存，直接返回
        if key in self._tf_cache:
            return self._tf_cache[key]
        T_MAP_TO_BODY = np.eye(4, dtype=np.float64)
        if not p.exists():
            print(f'get_T_MAP_TO_BODY: tf file not found: {p}, using identity')
            return T_MAP_TO_BODY
        try:
            obj = json.loads(p.read_text(encoding='utf-8'))
            tr = obj.get('translation', {})
            rot = obj.get('rotation', {})
            tx = float(tr.get('x', 0.0))
            ty = float(tr.get('y', 0.0))
            tz = float(tr.get('z', 0.0))
            qx = float(rot.get('x', 0.0))
            qy = float(rot.get('y', 0.0))
            qz = float(rot.get('z', 0.0))
            qw = float(rot.get('w', 1.0))
            # quaternion (qx,qy,qz,qw) -> rotation matrix (right-handed, w last)
            R = np.zeros((3, 3), dtype=np.float64)
            R[0, 0] = 1 - 2 * (qy * qy + qz * qz)
            R[0, 1] = 2 * (qx * qy - qz * qw)
            R[0, 2] = 2 * (qx * qz + qy * qw)
            R[1, 0] = 2 * (qx * qy + qz * qw)
            R[1, 1] = 1 - 2 * (qx * qx + qz * qz)
            R[1, 2] = 2 * (qy * qz - qx * qw)
            R[2, 0] = 2 * (qx * qz - qy * qw)
            R[2, 1] = 2 * (qy * qz + qx * qw)
            R[2, 2] = 1 - 2 * (qx * qx + qy * qy)
            T_MAP_TO_BODY = np.eye(4, dtype=np.float64)
            T_MAP_TO_BODY[:3, :3] = R
            T_MAP_TO_BODY[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
            try:
                pass
            except Exception:
                print(T_MAP_TO_BODY)
        except Exception as e:
            print(f'get_T_MAP_TO_BODY: failed to load TF {p}, using identity. err={e}')
        # 缓存并返回
        try:
            self._tf_cache[key] = T_MAP_TO_BODY
        except Exception:
            pass
        return T_MAP_TO_BODY

    def camera_to_map(self, point_cam, T_MAP_TO_BODY, count):
        """把相机坐标系下的点变换到 map 坐标系。

        输入:
            point_cam: 长度 3 的可迭代 (x,y,z)
            T_map_to_camera: 4x4 numpy 数组（map -> camera）
        返回:
            map_xyz: numpy 数组 shape (3,) 表示在 map 坐标系下的坐标
        """
        T_BODY_TO_CAMERA = np.array([
            [1, 0, 0, -0.05 ],
            [0, 1, 0, 0],
            [0, 0, 1, 0.04 ],
            [0, 0, 0 ,1]
        ], dtype=np.float64)

        # print(f"fuck {T_MAP_TO_BODY}")    
        T_BODY_TO_MAP_t = np.array([
            [1, 0, 0, -T_MAP_TO_BODY[0, 3]],
            [0, 1, 0, -T_MAP_TO_BODY[1, 3]],
            [0, 0, 1, -T_MAP_TO_BODY[2, 3]],
            [0, 0, 0 ,1]
        ], dtype=np.float64)

        T_MAP_TO_BODY_R = T_MAP_TO_BODY.copy()
        T_MAP_TO_BODY_R[:3, 3] = 0.0
        T_MAP_TO_BODY_R[0][1] = -T_MAP_TO_BODY_R[0][1]
        T_MAP_TO_BODY_R[1][0] = -T_MAP_TO_BODY_R[1][0]


        # if count % 30 == 0:
            # print("===============================")
            # print("T_MAP_TO_BODY_(old) =")
            # print(T_MAP_TO_BODY)
            # print("T_MAP_TO_BODY_R =")
            # print(T_MAP_TO_BODY_R)
            # print("T_BODY_TO_MAP_t =")
            # print(T_BODY_TO_MAP_t)
            # print("T_MAP_TO_BODY(new) =")
            # print(T_MAP_TO_BODY_R @ T_BODY_TO_MAP_t)
            # print("===============================")

        T_MAP_TO_CAMERA = T_BODY_TO_CAMERA @ T_MAP_TO_BODY_R @ T_BODY_TO_MAP_t
        p = np.asarray(point_cam, dtype=np.float64).reshape(3)
        T = np.asarray(T_MAP_TO_CAMERA, dtype=np.float64)
        if T.shape != (4,4):
            raise ValueError('transform_matrix must be 4x4')
        # 计算 camera -> map 的逆变换
        T_cam_to_map = inv(T)
        p_cam_h = np.array([p[0], p[1], p[2], 1.0], dtype=np.float64)
        p_map_h = T_cam_to_map @ p_cam_h

        return p_map_h[:3]

    def map_to_originmap_pixel(self, map_xyz):
        """把 map 坐标系下的 (x,y,z) 投影到 origin map 的像素 (u,v)。

        使用和 `map_cloud_on_originmap.py` 一样的投影规则（只用 x,y），并把 y 轴做垂直翻转以匹配图像坐标。
        返回 (u,v) 为整数像素坐标（可能超出图像范围）。
        """
        x_min, y_min = self.map_range[0]
        x_max, y_max = self.map_range[1]
        x, y = float(map_xyz[0]), float(map_xyz[1])
        if self.img_size is None:
            w, h = 1000, 600
        else:
            w, h = self.img_size
        u = int((x - x_min) / (x_max - x_min) * w)
        v = int(h - (y - y_min) / (y_max - y_min) * h)
        return (u, v)

    def calc_point_on_origin_map(self, pixel_point, T_map_to_body, count = 0):
        """
        封装 box->camera, camera->map, map->2dmap 的流程，返回结果 dict。
        参数：
            pixel_point: (u, v) 像素点
            T_map_to_body: 4x4 numpy 变换矩阵（map->body）
            count: 调试用计数
        返回 dict: {'cam_xyz':..., 'map_xyz':..., 'map_pixel':(u,v)}
        """
        u_arg = int(pixel_point[0]); v_arg = int(pixel_point[1])
        x = y = z = None
        try:
            calc_inst = calculate_xyz()
            x, y, z = calc_inst.calc_xyz(u_arg, v_arg)
        except Exception as e:
            raise RuntimeError(f'calc_xyz failed with args ({u_arg},{v_arg}): {e}')
        cam_xyz = np.array([x, y, z], dtype=np.float64)
        # 直接使用传入的 T_map_to_body
        if not (isinstance(T_map_to_body, np.ndarray) and T_map_to_body.shape == (4,4)):
            raise ValueError('T_map_to_body 必须是 4x4 numpy.ndarray')
        map_xyz = self.camera_to_map(cam_xyz, T_map_to_body, count)
        # print(f"map_xyz:{map_xyz}")
        map_pixel = self.map_to_originmap_pixel(map_xyz)
        # print(f"map_pixel:{map_pixel}")
        return {'cam_xyz': cam_xyz, 'map_xyz': map_xyz, 'map_pixel': map_pixel}

    def draw_point_on_map(self, points, origin_map_path: str = '/Users/xuzhenyu/Desktop/reutrn_stage/Pav-S_ws/src/costmap_process/scripts/red_zone_proj_assets/map_origin.jpg', color=(0,0,255), radius=1, out_path: str = None):
        """读入像素点并画到 origin_map 上并弹窗
        返回:
            绘制后的图片（numpy.ndarray）。
        """
        # 只接受像素点列表并在 origin_map 上可视化（弹窗），不做任何保存或尝试去读取相机图片。
        # points: 可迭代对象，元素为 (u, v) 或 [u, v]（像素坐标，origin_map 像素系）
        # 首先加载 origin_map 背景（若提供且存在），否则使用实例的 img_size 或默认画布
        img_path = Path(origin_map_path) if origin_map_path else self.map_img_path
        img = None
        if img_path is not None and Path(img_path).exists():
            img = cv2.imread(str(img_path))

        if img is None:
            raise ValueError("no img to draw")

        h, w = img.shape[:2]
        # debug: 打印加载的 origin_map 图片尺寸以及 transformer 内部记录的 img_size，方便排查坐标系不一致
        print(f'draw_point_on_map: loaded image {img_path} shape=(h={h}, w={w})')
        print(f'draw_point_on_map: transformer.img_size={self.img_size}')

        # 规范 points 为 numpy array, shape (N,2)
        pts_arr = np.asarray(points)
        if pts_arr.ndim == 1 and pts_arr.size == 2:
            pts_arr = pts_arr.reshape(1, 2)
        if pts_arr.ndim != 2 or pts_arr.shape[1] != 2:
            raise ValueError('points must be iterable of (u,v) pairs')

        # 打印 pts 的原始范围，便于排查
        try:
            print(f'draw_point_on_map: pts_arr min/max before swap: u [{pts_arr[:,0].min()},{pts_arr[:,0].max()}], v [{pts_arr[:,1].min()},{pts_arr[:,1].max()}]')
        except Exception:
            pass
        # 自动检测是否可能传入了 (v,u) 的颠倒顺序：如果多数第一列超出宽度但第二列在高度范围内，则交换
        first_col = pts_arr[:, 0]
        second_col = pts_arr[:, 1]
        count_first_oob = np.sum((first_col < 0) | (first_col >= w))
        count_second_in_h = np.sum((second_col >= 0) & (second_col < h))
        if count_first_oob > (len(pts_arr) * 0.6) and count_second_in_h > (len(pts_arr) * 0.6):
            # 可能传入 (v,u)，交换列
            pts_arr = pts_arr[:, ::-1]
            print('draw_point_on_map: swapped columns assuming (v,u) was passed')

        # 绘制每个像素点（假定已经是 origin_map 的像素坐标）
        drawn = []
        for pt in pts_arr:
            try:
                u = int(round(float(pt[0]))); v = int(round(float(pt[1])))
            except Exception:
                continue
            if 0 <= u < w and 0 <= v < h:
                # 先画黑色轮廓，再画目标颜色的填充，以保证对比度
                outline_radius = max(1, int(radius) + 1)
                try:
                    cv2.circle(img, (u, v), outline_radius, (0, 0, 0), -1)
                    cv2.circle(img, (u, v), int(radius), color, -1)
                except Exception:
                    # 在极少数情况下 cv2 绘制会失败，跳过该点
                    continue
                drawn.append((u, v))

        # 打印绘制统计和样例点
        if drawn:
            arr = np.array(drawn, dtype=int)
            print(f'draw_point_on_map: drew {len(drawn)} points. u range [{arr[:,0].min()},{arr[:,0].max()}], v range [{arr[:,1].min()},{arr[:,1].max()}]')
            print('example points:', drawn[:5])
        else:
            print('draw_point_on_map: no points drawn (all out of image bounds?)')

        # 若指定 out_path，则保存到文件
        if out_path:
            try:
                cv2.imwrite(str(out_path), img)
            except Exception:
                pass

        # 弹窗显示结果，按任意键关闭；不保存窗口则直接返回图像
        # cv2.imshow('points_on_origin_map', img)
        # cv2.waitKey(0)
        # cv2.destroyWindow('points_on_origin_map')       
        return img