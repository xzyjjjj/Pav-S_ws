from pathlib import Path
import json
import numpy as np
import threading
import time

# TODO: 保证获取的数据在时间上是同步的

from .config import LOCAL
try:
    from vision_msgs.msg import Detection2DArray
except Exception:
    Detection2DArray = None

_ROS_AVAILABLE = True
try:
    # delayed imports for ROS2
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    from tf2_ros import Buffer, TransformListener
except Exception:
    _ROS_AVAILABLE = False

class DataInterface:
    def __init__(self):
        # 本地/机器人模式选择
        self.use_ros = (not LOCAL) and _ROS_AVAILABLE
        # storage for last messages
        self._last_image = None
        self._last_detections = None
        self._image_event = threading.Event()
        self._det_event = threading.Event()
        # TF support
        self._tf_buffer = None
        self._tf_listener = None
        self._node = None
        self._bridge = None
        if self.use_ros:
            # initialize rclpy if not already
            try:
                if not rclpy.ok():
                    rclpy.init()
            except Exception:
                pass
            try:
                self._node = rclpy.create_node('data_interface_node')
            except Exception:
                self._node = Node('data_interface_node')
            self._bridge = CvBridge()
            try:
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self._node)
            except Exception:
                self._tf_buffer = None
                self._tf_listener = None
            # 订阅图像和检测
            try:
                self._node.create_subscription(Image, 'rgb_img', self._img_cb, 10)
                self._node.create_subscription(Detection2DArray, 'yolo_detections', self._det_cb, 10)
            except Exception:
                pass


    def read_bbox(self, bbox_path: str = '/Users/xuzhenyu/Desktop/reutrn_stage/Pav-S_ws/src/costmap_process/scripts/collected_data/collect1/000006_detections.json'):
        """
        读取 bbox 文件，返回所有检测框的四个角点和类别名：
        [ [(x1, y1), (x2, y2), (x3, y3), (x4, y4), class_id], ... ]
        参数:
            bbox_path: bbox json 路径
        返回:
            bboxes: list，每个元素为 [(x1, y1), (x2, y2), (x3, y3), (x4, y4), class_id]
        """

        p = Path(bbox_path)
        if not p.exists():
            raise FileNotFoundError(f"bbox json not found: {p}")

        obj = json.loads(p.read_text(encoding='utf-8'))
        dets = obj.get('detections', {})
        items = dets.get('_detections') or dets.get('detections') or []
        if not items:
            raise RuntimeError('no detections found in bbox json')

        bboxes = []
        for det in items:
            try:
                bbox = det.get('_bbox') or det.get('bbox') or {}
                center = bbox.get('_center') or bbox.get('center') or {}
                pos = center.get('_position') or center.get('position') or {}
                cx = pos.get('_x') or pos.get('x')
                cy = pos.get('_y') or pos.get('y')
                sx = bbox.get('_size_x') or bbox.get('size_x') or bbox.get('width')
                sy = bbox.get('_size_y') or bbox.get('size_y') or bbox.get('height')
                # 获取类别名
                class_id = None
                results = det.get('_results') or det.get('results') or []
                if results and isinstance(results, list):
                    hypo = results[0].get('_hypothesis') or results[0].get('hypothesis') or {}
                    class_id = hypo.get('_class_id') or hypo.get('class_id')
                if cx is None or cy is None or sx is None or sy is None or class_id is None:
                    continue
                cx = float(cx); cy = float(cy); sx = float(sx); sy = float(sy)
                x_min = cx - sx/2.0; x_max = cx + sx/2.0
                y_min = cy - sy/2.0; y_max = cy + sy/2.0
                bbox_point = [ (x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max), class_id ]
                bboxes.append(bbox_point)
            except Exception:
                continue
        return bboxes

    def get_all_type(self, bboxes):
        types = []
        for item in bboxes:
            if len(item) == 5:
                types.append(item[4])
        return types

    def get_specific_bbox(self, target_id, bboxes):
        """
        输入目标 id 和 read_bbox 的输出，返回该 id 对应的四个角点 [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]。
        若未找到则返回 None。
        """
        for item in bboxes:
            if len(item) == 5 and item[4] == target_id:
                return item[:4]
        return None
    
    def read_tf(self, tf_path = '/Users/xuzhenyu/Desktop/reutrn_stage/Pav-S_ws/src/costmap_process/scripts/collected_data/collect1/000006_tf.json'):
        """
        读取 tf 变换，输出4x4变换矩阵（numpy.ndarray）
        """
        p = Path(tf_path)
        T_MAP_TO_BODY = np.eye(4, dtype=np.float64)
        if not p.exists():
            print(f'read_tf: tf file not found: {p}, using identity')
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
            # quaternion (qx,qy,qz,qw) -> rotation matrix
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
            print("===============================")
            print('T_MAP_TO_BODY =')
            # 使用 numpy 的 array2string 以更友好的格式打印矩阵
            try:
                print(np.array2string(T_MAP_TO_BODY, precision=6, suppress_small=True))
            except Exception:
                # 兜底为通用打印
                print(T_MAP_TO_BODY)
            print("===============================")
        except Exception as e:
            print(f'read_tf: failed to load TF {p}, using identity. err={e}')
        return T_MAP_TO_BODY

    # ----------------------- ROS topic helpers -----------------------
    # def _img_cb(self, msg: 'Image'):
    #     """ROS Image callback: convert to BGR numpy and store"""
    #     try:
    #         img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #     except Exception:
    #         try:
    #             img = self._bridge.imgmsg_to_cv2(msg)
    #         except Exception:
    #             img = None
    #     if img is not None:
    #         self._last_image = img
    #         self._image_event.set()

    def _img_cb(self, msg: 'Image'):
        """ROS Image callback: convert to BGR numpy and store"""
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            try:
                img = self._bridge.imgmsg_to_cv2(msg)
            except Exception:
                img = None
        # 只在 img 有效且尺寸正常时写入缓冲
        if img is not None:
            try:
                # 有些情况下 img 可能不是 numpy 或 shape 有异常
                if hasattr(img, 'shape') and img.shape[0] > 0 and img.shape[1] > 0:
                    print("here reset img")
                    self._last_image = img
                    self._image_event.set()
                else:
                    print("[DataInterface] _img_cb received empty image, ignored")
            except Exception as e:
                print(f"[DataInterface] _img_cb validation failed: {e}")
        else:
            print("[DataInterface] _img_cb failed to convert msg to image (None)")
            
    def _det_cb(self, msg: 'Detection2DArray'):
        """ROS detection callback: 直接存储 Detection2DArray 消息对象。"""
        self._last_detections = msg
        self._det_event.set()

    def get_topic_data(self, timeout: float = 5.0):
        """
        Blocking: 等待从话题接收到一帧图像和一次检测 JSON，并返回 (img, detections_obj, T_map_to_body_matrix)

        如果没有启用 ROS（use_ros=False），抛出 RuntimeError。
        """
        if not self.use_ros:
            raise RuntimeError('ROS topic mode not enabled in DataInterface')

        # wait for image and detections (concurrently)
        start = time.time()
        while True:
            now = time.time()
            if self._image_event.is_set() and self._det_event.is_set():
                break
            if now - start > timeout:
                break
            # spin once to let callbacks run
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception:
                time.sleep(0.05)

        img = self._last_image
        dets = self._last_detections

        # try to get tf map->body
        T = np.eye(4, dtype=np.float64)
        try:
            if self._tf_buffer is not None:
                # try lookup latest transform map -> body
                from rclpy.time import Time
                try:
                    t = self._tf_buffer.lookup_transform('map', 'body', Time())
                    tr = t.transform.translation
                    rot = t.transform.rotation
                    tx, ty, tz = float(tr.x), float(tr.y), float(tr.z)
                    qx, qy, qz, qw = float(rot.x), float(rot.y), float(rot.z), float(rot.w)
                    # build rotation matrix
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
                    T = np.eye(4, dtype=np.float64)
                    T[:3, :3] = R
                    T[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
                except Exception:
                    # leave identity
                    pass
        except Exception:
            pass

        # 调试输出三个输入的结果
        print("[DataInterface] get_topic_data 输入结果:")
        if img is not None:
            print(f"  图像 shape: {img.shape}")
        else:
            print("  图像: None")
        if dets is not None:
            print(f"  检测 obj keys: {list(dets.keys()) if isinstance(dets, dict) else type(dets)}")
        else:
            print("  检测: None")
        print(f"  TF矩阵: \n{T}")
        self._image_event.clear()
        self._det_event.clear()
        return img, dets, T

    def parse_bbox_from_obj(self, obj):
        """将 Detection2DArray 消息对象解析为 [(x1, y1), (x2, y2), (x3, y3), (x4, y4), class_id] 列表。"""
        if obj is None or not hasattr(obj, 'detections'):
            return []
        bboxes = []
        for det in obj.detections:
            try:
                bbox = det.bbox
                center = bbox.center
                pos = center.position
                cx = pos.x
                cy = pos.y
                sx = bbox.size_x
                sy = bbox.size_y
                class_id = None
                if det.results and hasattr(det.results[0], 'hypothesis'):
                    class_id = det.results[0].hypothesis.class_id
                if cx is None or cy is None or sx is None or sy is None or class_id is None:
                    continue
                x_min = cx - sx/2.0; x_max = cx + sx/2.0
                y_min = cy - sy/2.0; y_max = cy + sy/2.0
                bbox_point = [ (x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max), class_id ]
                bboxes.append(bbox_point)
            except Exception:
                continue
        return bboxes