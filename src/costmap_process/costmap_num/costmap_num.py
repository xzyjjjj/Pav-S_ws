#!/usr/bin/env python3
"""costmap_num.py

ROS2 节点：订阅 YOLO 检测与投影图像，生成 8 个语义地图，
根据 mask 规则推断下一阶段并发布对应的 costmap 与 /next_stage。

该脚本可在没有 ROS2 环境时进行干运行（用于语法检查）。
"""

# TODO: 为 /num_x_map(x=1..8) 创建 OccupancyGrid 发布器

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

import cv2

try:  # ROS2 依赖（可能在当前环境中不存在）
	import rclpy
	from rclpy.node import Node
	from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
	from rclpy.duration import Duration
	from std_msgs.msg import Int32
	from sensor_msgs.msg import Image
	from nav_msgs.msg import OccupancyGrid
	from vision_msgs.msg import Detection2DArray, Detection2D
	from tf2_ros import Buffer, TransformListener, TransformException
except Exception:  # pragma: no cover - 仅用于离线语法检查
	rclpy = None
	Node = object  # type: ignore
	QoSProfile = object  # type: ignore
	ReliabilityPolicy = object  # type: ignore
	DurabilityPolicy = object  # type: ignore
	Duration = object  # type: ignore
	Int32 = object  # type: ignore
	Image = object  # type: ignore
	OccupancyGrid = object  # type: ignore
	Detection2DArray = object  # type: ignore
	Detection2D = object  # type: ignore
	ObjectHypothesisWithPose = object  # type: ignore
	Buffer = None  # type: ignore
	TransformListener = object  # type: ignore
	TransformException = Exception

from . import config as cfg


SEMANTIC_DIR = Path('semantic_map')
COSTMAP_DIR = Path('costmaps')

IMG_MAP = {
	1: '1_up.jpg',
	2: '1_down.jpg',
	3: '2_up.jpg',
	4: '2_down.jpg',
}

SEMANTIC_COLORS = np.array([
	[0, 0, 0],
	[0, 0, 255],
	[0, 255, 0],
	[255, 0, 0],
	[0, 255, 255],
	[255, 0, 255],
	[255, 255, 0],
	[128, 0, 255],
	[255, 128, 0],
], dtype=np.uint8)


@dataclass
class CameraIntrinsics:
	fx: float
	fy: float
	cx: float
	cy: float
	factor: float


def _load_gray(path: Path) -> np.ndarray:
	if not path.exists():
		raise FileNotFoundError(str(path))
	img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
	if img is None:
		raise RuntimeError(f'Failed to load image: {path}')
	return img


def _quaternion_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
	norm = math.sqrt(x * x + y * y + z * z + w * w)
	if norm == 0.0:
		return np.eye(3)
	x /= norm
	y /= norm
	z /= norm
	w /= norm
	xx, yy, zz = x * x, y * y, z * z
	xy, xz, yz = x * y, x * z, y * z
	wx, wy, wz = w * x, w * y, w * z
	return np.array([
		[1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
		[2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
		[2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
	])


def _build_transform(tx: float, ty: float, tz: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
	cr, sr = math.cos(roll), math.sin(roll)
	cp, sp = math.cos(pitch), math.sin(pitch)
	cy, sy = math.cos(yaw), math.sin(yaw)
	# ROS RPY: R = Rz * Ry * Rx
	R = np.array([
		[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
		[sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
		[-sp, cp * sr, cp * cr],
	])
	T = np.eye(4)
	T[:3, :3] = R
	T[:3, 3] = np.array([tx, ty, tz])
	return T


class CostmapNumNode(Node):
	def __init__(self) -> None:
		node_name = 'costmap_num'
		if rclpy is not None:
			super().__init__(node_name)
		else:  # pragma: no cover - 非 ROS 环境下的占位属性
			self.get_logger = lambda: None

		self.map_frame = cfg.STATIC_TRANSFORM.get('parent_frame', 'map')
		self.static_child_frame = cfg.STATIC_TRANSFORM.get('child_frame', 'camera_init')
		self.camera_frame_candidates: List[str] = []
		# 优先使用 camera_link，其次使用配置中的 child_frame（默认 camera_init）
		for candidate in ('camera_link', self.static_child_frame):
			if candidate and candidate not in self.camera_frame_candidates:
				self.camera_frame_candidates.append(candidate)
		self.camera_frame_default = self.camera_frame_candidates[0] if self.camera_frame_candidates else self.static_child_frame
		cam_cfg = cfg.CAMERA
		self.intrinsics = CameraIntrinsics(
			fx=float(cam_cfg['fx']),
			fy=float(cam_cfg['fy']),
			cx=float(cam_cfg['cx']),
			cy=float(cam_cfg['cy']),
			factor=float(cam_cfg.get('factor', 1000.0)),
		)

		st = cfg.STATIC_TRANSFORM
		self.T_map_to_cam = _build_transform(
			st['tx'], st['ty'], st['tz'], st['roll'], st['pitch'], st['yaw']
		)
		self.T_cam_to_map = np.linalg.inv(self.T_map_to_cam)

		self.semantic_labels = list(range(1, 9))
		self.mask1 = _load_gray(SEMANTIC_DIR / 'mask1.jpg') if (SEMANTIC_DIR / 'mask1.jpg').exists() else None
		self.mask2 = _load_gray(SEMANTIC_DIR / 'mask2.jpg') if (SEMANTIC_DIR / 'mask2.jpg').exists() else None

		self.semantic_shape = self._determine_semantic_shape()
		self.semantic_maps: List[np.ndarray] = [
			np.zeros(self.semantic_shape, dtype=np.uint8) for _ in range(8)
		]

		self.map_roi_min = np.array(cfg.MAP_ROI_MIN, dtype=np.float32)
		self.map_roi_max = np.array(cfg.MAP_ROI_MAX, dtype=np.float32)
		self.map_size = np.array([self.semantic_shape[1], self.semantic_shape[0]], dtype=np.int32)

		self.latest_depth_image = None
		self.latest_depth_encoding = None
		self.latest_depth_header = None
		self.latest_rgb_image = None
		self.latest_rgb_encoding = None
		self.latest_rgb_header = None

		self.current_stage = None
		self.next_stage_value = 0

		self.costmap_publish_rate = 1.0
		self.qos_profile = self._make_qos()

		self.costmap_images = self._load_costmap_images()
		self.semantic_visualization = np.zeros((*self.semantic_shape, 3), dtype=np.uint8)

		self.tf_buffer = None
		self.tf_listener = None
		if rclpy is not None and Buffer is not None:
			self.tf_buffer = Buffer()
			self.tf_listener = TransformListener(self.tf_buffer, self)

		if rclpy is not None:
			self.detection_sub = self.create_subscription(
				Detection2DArray, '/yolo_detections', self.detections_callback, self.qos_profile
			)
			self.depth_image_sub = self.create_subscription(
				Image, '/depth_img', self.depth_image_callback, self.qos_profile
			)
			self.rgb_image_sub = self.create_subscription(
				Image, '/rgb_img', self.rgb_image_callback, self.qos_profile
			)
			self.stage_sub = self.create_subscription(
				Int32, '/stage', self.stage_callback, self.qos_profile
			)

			self.next_stage_pub = self.create_publisher(Int32, '/next_stage', 10)
			self.costmap_pub = self.create_publisher(OccupancyGrid, 'processed_costmap_num', 10)
			self.semantic_viz_pub = self.create_publisher(Image, 'semantic_map_visualization', 10)

			self.timer = self.create_timer(1.0 / self.costmap_publish_rate, self._publish_loop)

	# -------------------------- 初始化辅助 --------------------------
	def _determine_semantic_shape(self) -> Tuple[int, int]:
		for idx in range(1, 9):
			path = SEMANTIC_DIR / f'{idx}.jpg'
			if path.exists():
				img = _load_gray(path)
				return img.shape
		if self.mask1 is not None:
			return self.mask1.shape
		if self.mask2 is not None:
			return self.mask2.shape
		return (400, 400)

	def _make_qos(self):
		if rclpy is None:
			return 10
		profile = QoSProfile(depth=10)
		profile.reliability = ReliabilityPolicy.BEST_EFFORT
		profile.durability = DurabilityPolicy.VOLATILE
		return profile

	def _load_costmap_images(self) -> Dict[int, np.ndarray]:
		base = Path(os.path.dirname(os.path.realpath(__file__))) / COSTMAP_DIR
		maps: Dict[int, np.ndarray] = {}
		for stage, name in IMG_MAP.items():
			path = base / name if not os.path.isabs(name) else Path(name)
			img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
			if img is not None:
				maps[stage] = img
		return maps

	# -------------------------- ROS 回调 --------------------------
	def depth_image_callback(self, msg: Any) -> None:
		array, encoding = self._image_msg_to_numpy(msg)
		if array is not None:
			self.latest_depth_image = array
			self.latest_depth_encoding = encoding
			self.latest_depth_header = msg.header if hasattr(msg, 'header') else None

	def rgb_image_callback(self, msg: Any) -> None:
		array, encoding = self._image_msg_to_numpy(msg)
		if array is not None:
			self.latest_rgb_image = array
			self.latest_rgb_encoding = encoding
			self.latest_rgb_header = msg.header if hasattr(msg, 'header') else None

	def detections_callback(self, msg: Any) -> None:
		if self.latest_depth_image is None:
			return
		depth_frame_id = getattr(self.latest_depth_header, 'frame_id', '') if self.latest_depth_header is not None else ''
		raw_frame_id = getattr(msg.header, 'frame_id', '') if hasattr(msg, 'header') else ''
		frame_id = raw_frame_id or depth_frame_id or self.camera_frame_default
		det_count = len(getattr(msg, 'detections', []))
		if rclpy is not None:
			self.get_logger().info(f"/yolo_detections received: {det_count} detections (frame: {frame_id})")
		else:
			print(f"/yolo_detections received: {det_count} detections (frame: {frame_id})")
		original_frame_id = raw_frame_id or depth_frame_id
		for det in getattr(msg, 'detections', []):
			cls_id = self._extract_class_id(det)
			if cls_id is None or not (1 <= cls_id <= 8):
				continue
			bbox = det.bbox
			points_cam = self._points_from_bbox(bbox)
			if points_cam is None or points_cam.size == 0:
				continue
			points_map = self._transform_points(points_cam, frame_id, original_frame_id)
			if points_map is None or points_map.size == 0:
				continue
			self._update_semantic_map(cls_id - 1, points_map)
		self._evaluate_next_stage()

	def stage_callback(self, msg: Any) -> None:
		self.current_stage = int(msg.data)
		self._evaluate_next_stage()

	# -------------------------- 核心逻辑 --------------------------
	def _publish_loop(self) -> None:
		self._publish_next_stage()
		self._publish_costmap()
		self._publish_semantic_visualization()

	def _publish_next_stage(self) -> None:
		if rclpy is None:
			return
		msg = Int32()
		msg.data = int(self.next_stage_value)
		self.next_stage_pub.publish(msg)

	def _publish_costmap(self) -> None:
		if rclpy is None:
			return
		stage = int(self.next_stage_value)
		if stage not in self.costmap_images:
			return
		grid = self._image_to_occupancy(self.costmap_images[stage])
		if grid is None:
			return
		self.costmap_pub.publish(grid)

	def _publish_semantic_visualization(self) -> None:
		if rclpy is None:
			return
		if not hasattr(self, 'semantic_viz_pub'):
			return
		viz = self._compose_semantic_visualization()
		msg = Image()
		msg.height, msg.width = viz.shape[:2]
		msg.encoding = 'bgr8'
		msg.step = viz.shape[1] * viz.shape[2]
		msg.data = viz.tobytes()
		if hasattr(msg, 'header'):
			if rclpy is not None:
				msg.header.stamp = self.get_clock().now().to_msg()
			msg.header.frame_id = self.map_frame
		self.semantic_viz_pub.publish(msg)

	def _compose_semantic_visualization(self) -> np.ndarray:
		label_map = np.zeros(self.semantic_shape, dtype=np.uint8)
		for idx, sm in enumerate(self.semantic_maps, start=1):
			mask = sm == 255
			label_map[mask] = idx
		max_label = int(label_map.max())
		if max_label >= SEMANTIC_COLORS.shape[0]:
			extra = np.tile(SEMANTIC_COLORS[-1:], (max_label - SEMANTIC_COLORS.shape[0] + 1, 1))
			colors = np.concatenate([SEMANTIC_COLORS, extra], axis=0)
		else:
			colors = SEMANTIC_COLORS
		colorized = colors[label_map]
		self.semantic_visualization[:, :] = colorized
		return self.semantic_visualization

	def _image_to_occupancy(self, image: np.ndarray) -> Optional[Any]:
		if OccupancyGrid is object:
			return None
		grid = OccupancyGrid()
		if rclpy is not None:
			grid.header.stamp = self.get_clock().now().to_msg()
		grid.header.frame_id = 'map'
		h, w = image.shape
		grid.info.resolution = 1.0
		grid.info.width = int(w)
		grid.info.height = int(h)
		grid.info.origin.position.x = 0.0
		grid.info.origin.position.y = 0.0
		grid.info.origin.position.z = 0.0
		grid.info.origin.orientation.w = 1.0

		flat = image.flatten()
		occupancy = np.empty_like(flat, dtype=np.int8)
		occupancy[flat >= 128] = 0
		occupancy[flat < 128] = 100
		grid.data = occupancy.tolist()
		return grid

	def _evaluate_next_stage(self) -> None:
		if self.current_stage is None:
			return
		if self.current_stage == 0:
			processed = self._apply_mask(self.mask1)
			choice = self._decide_stage(processed)
			if choice is True:
				self.next_stage_value = 1
			elif choice is False:
				self.next_stage_value = 2
		elif self.current_stage in (1, 2):
			processed = self._apply_mask(self.mask2)
			choice = self._decide_stage(processed)
			if choice is True:
				self.next_stage_value = 3
			elif choice is False:
				self.next_stage_value = 4

	def _apply_mask(self, mask_img: Optional[np.ndarray]) -> List[np.ndarray]:
		if mask_img is None:
			return [m.copy() for m in self.semantic_maps]
		_, mask_bin = cv2.threshold(mask_img, 127, 255, cv2.THRESH_BINARY_INV)
		if mask_bin.shape != self.semantic_shape:
			mask_bin = cv2.resize(mask_bin, (self.semantic_shape[1], self.semantic_shape[0]), interpolation=cv2.INTER_NEAREST)
		processed: List[np.ndarray] = []
		for sm in self.semantic_maps:
			if sm.shape != mask_bin.shape:
				im = cv2.resize(sm, (mask_bin.shape[1], mask_bin.shape[0]), interpolation=cv2.INTER_NEAREST)
			else:
				im = sm
			processed.append(cv2.bitwise_and(im, mask_bin))
		return processed

	def _decide_stage(self, processed: Sequence[np.ndarray]) -> Optional[bool]:
		areas = [int(np.count_nonzero(p == 255)) for p in processed]
		valid_idx = [i for i, a in enumerate(areas) if a > 0]
		if len(valid_idx) == 0:
			return None
		chosen = sorted(valid_idx, key=lambda i: areas[i], reverse=True)[:2]
		if len(chosen) < 2:
			return None
		centroids: List[Tuple[float, float]] = []
		for idx in chosen:
			ys, xs = np.where(processed[idx] == 255)
			if len(xs) == 0:
				centroids.append((float('inf'), float('inf')))
			else:
				centroids.append((float(xs.mean()), float(ys.mean())))
		upper_idx, lower_idx = (0, 1) if centroids[0][1] <= centroids[1][1] else (1, 0)
		upper_val = self.semantic_labels[chosen[upper_idx]]
		lower_val = self.semantic_labels[chosen[lower_idx]]
		return upper_val > lower_val

	# -------------------------- 投影相关 --------------------------
	def _points_from_bbox(self, bbox) -> Optional[np.ndarray]:
		img = self.latest_depth_image
		if img is None:
			return None
		h, w = img.shape[:2]
		cx = getattr(bbox.center, 'position', getattr(bbox.center, 'pose', bbox.center)).x if hasattr(bbox, 'center') else bbox.center_x
		cy = getattr(bbox.center, 'position', getattr(bbox.center, 'pose', bbox.center)).y if hasattr(bbox, 'center') else bbox.center_y
		size_x = getattr(bbox, 'size_x', getattr(bbox, 'width', 0.0))
		size_y = getattr(bbox, 'size_y', getattr(bbox, 'height', 0.0))
		x1 = max(0, int(math.floor(cx - size_x / 2.0)))
		x2 = min(w - 1, int(math.ceil(cx + size_x / 2.0)))
		y1 = max(0, int(math.floor(cy - size_y / 2.0)))
		y2 = min(h - 1, int(math.ceil(cy + size_y / 2.0)))
		if x1 >= x2 or y1 >= y2:
			return None

		region = img[y1:y2, x1:x2]
		if region.size == 0:
			return None

		if region.ndim == 3 and region.shape[2] >= 3 and self.latest_depth_encoding in {'32FC3', '64FC3'}:
			pts = region.reshape(-1, region.shape[2])[:, :3]
			mask = np.isfinite(pts).all(axis=1)
			pts = pts[mask]
		else:
			if region.ndim == 3 and region.shape[2] == 1:
				depth = region[:, :, 0]
			else:
				depth = region
			depth = depth.astype(np.float32)
			depth[depth <= 0.0] = np.nan
			ys, xs = np.mgrid[y1:y2, x1:x2]
			xs = xs.flatten().astype(np.float32)
			ys = ys.flatten().astype(np.float32)
			depth_flat = depth.flatten()
			valid = np.isfinite(depth_flat) & (depth_flat > 0)
			xs = xs[valid]
			ys = ys[valid]
			depth_flat = depth_flat[valid]
			if depth_flat.size == 0:
				return None
			z = depth_flat / float(self.intrinsics.factor)
			x = (xs - self.intrinsics.cx) * z / self.intrinsics.fx
			y = (ys - self.intrinsics.cy) * z / self.intrinsics.fy
			pts = np.stack([x, y, z], axis=1)
		return pts

	def _transform_points(self, pts: np.ndarray, frame_id: str, original_frame_id: str) -> Optional[np.ndarray]:
		if pts.size == 0:
			return None
		candidate_frames = self._candidate_frames(frame_id)
		if self.tf_buffer is not None and rclpy is not None:
			for candidate in candidate_frames:
				try:
					transform = self.tf_buffer.lookup_transform(
						self.map_frame,
						candidate,
						rclpy.time.Time(),
						timeout=Duration(seconds=0.1),
					)
					R = _quaternion_to_rot_matrix(
						transform.transform.rotation.x,
						transform.transform.rotation.y,
						transform.transform.rotation.z,
						transform.transform.rotation.w,
					)
					t = np.array([
						transform.transform.translation.x,
						transform.transform.translation.y,
						transform.transform.translation.z,
					])
					pts_map = (R @ pts.T).T + t
					return pts_map
				except TransformException:
					continue
		if self.static_child_frame and original_frame_id in ('', self.static_child_frame) and self.static_child_frame in candidate_frames:
			homo = np.concatenate([pts, np.ones((pts.shape[0], 1))], axis=1)
			pts_map_h = (self.T_cam_to_map @ homo.T).T
			return pts_map_h[:, :3]
		return None

	def _candidate_frames(self, preferred: Optional[str]) -> List[str]:
		frames: List[str] = []
		if preferred:
			frames.append(preferred)
		for fallback in self.camera_frame_candidates:
			if fallback and fallback not in frames:
				frames.append(fallback)
		return frames

	def _update_semantic_map(self, idx: int, points_map: np.ndarray) -> None:
		if idx < 0 or idx >= len(self.semantic_maps):
			return
		x_min, y_min = self.map_roi_min
		x_max, y_max = self.map_roi_max
		width, height = self.map_size
		if width <= 1 or height <= 1:
			return
		xs = points_map[:, 0]
		ys = points_map[:, 1]
		mask = (
			(xs >= x_min)
			& (xs <= x_max)
			& (ys >= y_min)
			& (ys <= y_max)
		)
		xs = xs[mask]
		ys = ys[mask]
		if xs.size == 0:
			return
		u = ((xs - x_min) / (x_max - x_min) * (width - 1)).astype(np.int32)
		v = ((ys - y_min) / (y_max - y_min) * (height - 1)).astype(np.int32)
		v = np.clip(v, 0, height - 1)
		u = np.clip(u, 0, width - 1)
		self.semantic_maps[idx][height - 1 - v, u] = 255

	def _extract_class_id(self, det: Any) -> Optional[int]:
		if det is None:
			return None
		results = getattr(det, 'results', [])
		if not results:
			return None
		best = max(results, key=lambda r: getattr(r, 'score', 0.0))
		hyp = getattr(best, 'hypothesis', best)
		cls_id = getattr(hyp, 'class_id', getattr(hyp, 'id', None))
		if cls_id is None:
			return None
		try:
			return int(cls_id)
		except Exception:
			try:
				return int(float(cls_id))
			except Exception:
				return None

	# -------------------------- 工具函数 --------------------------
	def _image_msg_to_numpy(self, msg: Any) -> Tuple[Optional[np.ndarray], Optional[str]]:
		encoding = getattr(msg, 'encoding', '32FC3')
		height = getattr(msg, 'height', 0)
		width = getattr(msg, 'width', 0)
		if height == 0 or width == 0:
			return None, None
		data = getattr(msg, 'data', b'')
		step = getattr(msg, 'step', 0)
		if step == 0:
			step = len(data) // height if height else 0
		if encoding in {'rgb8', 'bgr8', 'rgba8', 'bgra8'}:
			channels = 4 if 'a8' in encoding else 3
			dtype = np.uint8
			arr = np.frombuffer(data, dtype=dtype)
			if arr.size < height * width * channels:
				return None, None
			arr = arr.reshape((height, width, channels))
			return arr.copy(), encoding
		if encoding in {'32FC3', '32FC4'}:
			dtype = np.float32
			channels = 4 if encoding == '32FC4' else 3
			arr = np.frombuffer(data, dtype=dtype)
			if arr.size < height * width * channels:
				return None, None
			arr = arr.reshape((height, width, channels))
			return arr[:, :, :3].copy(), encoding
		if encoding in {'64FC3', '64FC4'}:
			dtype = np.float64
			channels = 4 if encoding == '64FC4' else 3
			arr = np.frombuffer(data, dtype=dtype)
			if arr.size < height * width * channels:
				return None, None
			arr = arr.reshape((height, width, channels))
			return arr[:, :, :3].astype(np.float32), encoding
		if encoding in {'32FC1', '64FC1'}:
			dtype = np.float32 if encoding == '32FC1' else np.float64
			arr = np.frombuffer(data, dtype=dtype)
			if arr.size < height * width:
				return None, None
			arr = arr.reshape((height, width))
			if dtype != np.float32:
				arr = arr.astype(np.float32)
			return arr, encoding
		if encoding in {'mono16'}:
			arr = np.frombuffer(data, dtype=np.uint16)
			if arr.size < height * width:
				return None, None
			arr = arr.reshape((height, width)).astype(np.float32)
			return arr, encoding
		if encoding in {'mono8', '8UC1'}:
			arr = np.frombuffer(data, dtype=np.uint8)
			if arr.size < height * width:
				return None, None
			arr = arr.reshape((height, width)).astype(np.float32)
			return arr, encoding
		# 未知编码，尝试按 step 读取
		if step % width == 0:
			channels = step // width
			arr = np.frombuffer(data, dtype=np.uint8)
			if arr.size < height * step:
				return None, None
			arr = arr.reshape((height, step))[:, : width * channels]
			arr = arr.reshape((height, width, channels))
			return arr.astype(np.float32), encoding
		return None, None


def main(args: Optional[Sequence[str]] = None) -> int:
	if rclpy is None:
		# Dry run: instantiate node for syntax check
		CostmapNumNode()
		print('Dry-run completed (ROS2 not available).')
		return 0

	rclpy.init(args=args)
	node = CostmapNumNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()
	return 0


if __name__ == '__main__':
	import sys

	sys.exit(main(sys.argv))
