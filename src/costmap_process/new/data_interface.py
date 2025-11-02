"""
perception_data_utils.py

本模块提供数据处理的辅助函数，包括：
1. 从本地 JSON 文件读取 bbox 和 tf。
2. 从 ROS 2 Detection2DArray 消息中解析 bbox。
3. 提供坐标变换相关的数学计算。

(此文件重构自 data_interface.py，移除了所有 ROS 节点和同步逻辑)
"""

import json
import numpy as np
from pathlib import Path

# 尝试导入 vision_msgs，如果失败则在 ROS 模式下会出错
try:
    from vision_msgs.msg import Detection2DArray
except Exception:
    Detection2DArray = None

def tf_matrix_from_tr_and_quat(tr_vec, quat_vec):
    """
    根据平移向量和四元数向量计算 4x4 齐次变换矩阵。
    tr_vec: [tx, ty, tz]
    quat_vec: [qx, qy, qz, qw]
    """
    tx, ty, tz = tr_vec
    qx, qy, qz, qw = quat_vec
    
    T = np.eye(4, dtype=np.float64)
    
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

    T[:3, :3] = R
    T[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
    return T

def read_bbox(bbox_path: str):
    """
    读取 bbox 文件，返回所有检测框的四个角点和类别名：
    [ [(x1, y1), (x2, y2), (x3, y3), (x4, y4), class_id], ... ]
    """
    p = Path(bbox_path)
    if not p.exists():
        print(f"[DataUtils] 警告: bbox json 未找到: {p}，返回空列表")
        return []

    try:
        obj = json.loads(p.read_text(encoding='utf-8'))
        dets = obj.get('detections', {})
        items = dets.get('_detections') or dets.get('detections') or []
        if not items:
            print("[DataUtils] 警告: bbox json 中未找到 'detections'，返回空列表")
            return []
    except Exception as e:
        print(f"[DataUtils] 错误: 读取或解析 bbox json 失败: {e}，返回空列表")
        return []

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

def read_tf(tf_path: str):
    """
    读取 tf 变换，输出4x4变换矩阵（numpy.ndarray）
    """
    p = Path(tf_path)
    T_MAP_TO_BODY = np.eye(4, dtype=np.float64)
    if not p.exists():
        print(f'[DataUtils] 警告: tf file not found: {p}, using identity')
        return T_MAP_TO_BODY
    try:
        obj = json.loads(p.read_text(encoding='utf-8'))
        tr = obj.get('translation', {})
        rot = obj.get('rotation', {})
        tr_vec = [
            float(tr.get('x', 0.0)),
            float(tr.get('y', 0.0)),
            float(tr.get('z', 0.0))
        ]
        quat_vec = [
            float(rot.get('x', 0.0)),
            float(rot.get('y', 0.0)),
            float(rot.get('z', 0.0)),
            float(rot.get('w', 1.0))
        ]
        T_MAP_TO_BODY = tf_matrix_from_tr_and_quat(tr_vec, quat_vec)
        
        print("===============================")
        print(f'T_MAP_TO_BODY (from {p.name}) =')
        try:
            print(np.array2string(T_MAP_TO_BODY, precision=6, suppress_small=True))
        except Exception:
            print(T_MAP_TO_BODY)
        print("===============================")
    except Exception as e:
        print(f'[DataUtils] 错误: failed to load TF {p}, using identity. err={e}')
    return T_MAP_TO_BODY

def get_all_type(bboxes_list):
    """从解析后的 bboxes 列表中提取所有唯一的 class_id"""
    types = set()
    for item in bboxes_list:
        if len(item) == 5:
            types.add(item[4])
    return list(types)

def get_specific_bbox(target_id, bboxes_list):
    """
    输入目标 id 和 bboxes 列表，返回该 id 对应的四个角点 [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]。
    若未找到则返回 None。
    """
    for item in bboxes_list:
        if len(item) == 5 and item[4] == target_id:
            return item[:4]
    return None

def parse_bbox_from_obj(obj: 'Detection2DArray'):
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