#!/usr/bin/env python3
"""judge_state.py

- 提供函数 process_semantic_maps，用于：
- 接受 8 个语义图（每张图像中黑色(0)表示该语义在该像素处）
- 接受一个掩码图（掩码中黑色(0)表示需要保留的区域）
- 对每张语义图应用掩码后，按颜色叠加显示

函数签名：
	process_semantic_maps(sem_maps, mask, show=True, save_path=None)

参数说明：
 - sem_maps: list 长度为 8 的元素，每个元素可为文件路径 (str) 或 numpy 数组 (H,W) 灰度图
 - mask: 文件路径或 numpy 数组 (H,W) 灰度图，只有 mask==255 的位置才保留语义
 - show: 是否用 OpenCV 弹窗显示（默认 True）
 - save_path: 如果给定，则会把合成图和瓦片图保存到该路径（文件名前缀）

返回： (combined_color, processed_list)
 - combined_color: HxWx3 的 BGR 彩色叠加图（numpy uint8）
 - processed_list: 长度 8 的灰度处理后数组列表（每个仅含 0/255）

"""

from typing import List, Tuple, Union, Optional
import os
import sys
import argparse
import cv2
import numpy as np


ImageLike = Union[str, np.ndarray]


def _load_gray(img: ImageLike) -> np.ndarray:
	"""加载灰度图：如果传入路径则读取文件，否则假定已是 numpy 数组并返回拷贝。"""
	if isinstance(img, str):
		if not os.path.exists(img):
			raise FileNotFoundError(f"Image path not found: {img}")
		im = cv2.imread(img, cv2.IMREAD_GRAYSCALE)
		if im is None:
			raise RuntimeError(f"Failed to read image: {img}")
		return im
	elif isinstance(img, np.ndarray):
		if img.ndim == 3:
			# convert to gray
			return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		return img.copy()
	else:
		raise TypeError('img must be path or numpy.ndarray')


def process_semantic_maps(sem_maps: List[ImageLike], mask: ImageLike,
						  show: bool = True, save_path: Optional[str] = None
						  ) -> Tuple[np.ndarray, List[np.ndarray]]:
	"""处理 8 个语义图并显示/返回叠加结果。

	每张语义图中黑色(0)表示该语义存在。掩码中黑色(0)表示需要保留的区域。
	处理步骤：
	- 加载所有图像并确保尺寸一致（按第一张图尺寸为基准，其他图会被缩放）
	- 对每张语义图应用掩码（只有 mask==255 的像素保留，否则置 0）
	- 为每张语义图分配一种颜色并将它们叠加到彩色图上（颜色通过相加方式混合）

	返回合成的 BGR 彩色图与每张处理后的灰度图列表。
	"""
	if len(sem_maps) != 8:
		raise ValueError('sem_maps must contain exactly 8 items')

	# Load mask
	mask_img = _load_gray(mask)
	# In your dataset, black (0) means KEEP and white (255) means discard.
	# We want mask_bin where 255 indicates valid/keep pixels.
	# So invert the typical threshold: pixels <=127 -> 255 (keep), >127 -> 0 (discard)
	_, mask_bin = cv2.threshold(mask_img, 127, 255, cv2.THRESH_BINARY_INV)

	# Load first semantic map to get target size
	first = _load_gray(sem_maps[0])
	h, w = first.shape

	# Resize mask to match first if needed
	if mask_bin.shape != (h, w):
		mask_bin = cv2.resize(mask_bin, (w, h), interpolation=cv2.INTER_NEAREST)

	processed = []
	for idx, sm in enumerate(sem_maps):
		im = _load_gray(sm)
		if im.shape != (h, w):
			im = cv2.resize(im, (w, h), interpolation=cv2.INTER_NEAREST)
		# Threshold to binary semantic presence where black (0) -> presence (255)
		_, im_bin = cv2.threshold(im, 127, 255, cv2.THRESH_BINARY_INV)
		# Apply mask: only keep where mask==255 (mask was inverted earlier so 255 means keep)
		im_masked = cv2.bitwise_and(im_bin, mask_bin)
		processed.append(im_masked)

	# Prepare color palette (BGR) for 8 semantics — visually distinct
	palette = [
		(0, 0, 255),    # red
		(0, 128, 255),  # orange
		(0, 255, 255),  # yellow
		(0, 255, 0),    # green
		(255, 0, 0),    # blue
		(255, 0, 128),  # purple
		(128, 0, 255),  # magenta-ish
		(255, 255, 0),  # cyan-ish
	]

	# Create combined color image by summing color contributions
	combined = np.zeros((h, w, 3), dtype=np.uint16)  # use uint16 to avoid overflow during add
	for i, im_masked in enumerate(processed):
		# normalize mask to 0/1
		m = (im_masked == 255).astype(np.uint16)[:, :, None]
		color = np.array(palette[i], dtype=np.uint16)[None, None, :]
		combined += m * color

	# Clip to 0..255 and convert to uint8
	combined = np.clip(combined, 0, 255).astype(np.uint8)

	# Also create a 2x4 tiled visualization showing each processed semantic (colorized)
	tile_h, tile_w = h // 2, w // 4
	# If too small, keep original size for tiles
	if tile_h < 10 or tile_w < 10:
		tile_h, tile_w = h, w

	tiles = []
	for i, im_masked in enumerate(processed):
		resized = cv2.resize(im_masked, (tile_w, tile_h), interpolation=cv2.INTER_NEAREST)
		# colorize
		color = np.array(palette[i], dtype=np.uint8)
		colored = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
		mask_bool = (resized == 255)
		for c in range(3):
			colored[:, :, c][mask_bool] = color[c]
		tiles.append(colored)

	# build 2x4 grid
	row1 = np.hstack(tiles[0:4])
	row2 = np.hstack(tiles[4:8])
	grid = np.vstack([row1, row2])

	# Show only the semantics grid if requested. Do NOT show combined_overlay (mask) window.
	if show:
		cv2.namedWindow('semantics_grid', cv2.WINDOW_NORMAL)
		cv2.imshow('semantics_grid', grid)
		print('Press any key on the image window to continue...')
		cv2.waitKey(0)
		cv2.destroyAllWindows()

	# Save if requested
	if save_path is not None:
		base = save_path
		cv2.imwrite(base + '_combined.png', combined)
		cv2.imwrite(base + '_grid.png', grid)

	# convert processed list to uint8 0/255
	processed_out = [p.astype(np.uint8) for p in processed]
	return combined, processed_out


if __name__ == '__main__':
	# 简单命令行测试：尝试从 ./semantic_map 中读取 8 张语义图片（按文件名排序），以及 mask.jpg
	import glob
	base_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'semantic_map')
	files = sorted(glob.glob(os.path.join(base_dir, '*')))
	# pick image files excluding mask.jpg and require basenames to be integers 1..8
	candidate_imgs = [f for f in files if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')) and os.path.basename(f).lower() != 'mask.jpg']
	if len(candidate_imgs) == 0:
		print('No semantic images found in', base_dir)
		sys.exit(1)

	# map from integer name -> path for names 1..8
	num_map = {}
	for f in candidate_imgs:
		name = os.path.splitext(os.path.basename(f))[0]
		if name.isdigit():
			v = int(name)
			if 1 <= v <= 8:
				num_map[v] = f

	missing = [i for i in range(1, 9) if i not in num_map]
	if missing:
		print(f'Missing semantic map files for indices: {missing}. Expected files named 1..8 (without extension) in {base_dir}')
		sys.exit(1)

	# build sem_maps ordered 1..8
	sem_maps = [num_map[i] for i in range(1, 9)]

	# Parse optional start state: 0 (start with mask1), or 1/2 (skip stage1 and start with mask2)
	parser = argparse.ArgumentParser()
	parser.add_argument('--start-state', type=int, choices=[0,1,2], default=0, help='Initial state: 0 (use mask1 first), 1 or 2 (skip mask1, start with mask2)')
	args = parser.parse_args()
	start_state = int(args.start_state)
	state = start_state

	def run_with_mask(mask_filename: str):
		mask_path_local = os.path.join(base_dir, mask_filename)
		if not os.path.exists(mask_path_local):
			print(f'No {mask_filename} found; assuming all pixels are KEEP for this mask')
			first_img = _load_gray(sem_maps[0])
			mask_arr_local = np.zeros_like(first_img, dtype=np.uint8)
		else:
			mask_arr_local = mask_path_local
		combined_local, processed_local = process_semantic_maps(sem_maps, mask_arr_local, show=False, save_path=None)
		return combined_local, processed_local

	# Stage 1: state == 0 use mask1
	if start_state == 0:
		combined, processed = run_with_mask('mask1.jpg')

		# create tiles with filenames overlaid
	else:
		# skip stage1, directly prepare processed using mask2
		combined, processed = run_with_mask('mask2.jpg')

	# create tiles with filenames overlaid
	# reuse palette
	palette = [
		(0, 0, 255),    # red
		(0, 128, 255),  # orange
		(0, 255, 255),  # yellow
		(0, 255, 0),    # green
		(255, 0, 0),    # blue
		(255, 0, 128),  # purple
		(128, 0, 255),  # magenta-ish
		(255, 255, 0),  # cyan-ish
	]

	h, w = combined.shape[:2]
	tile_h, tile_w = h // 2, w // 4
	if tile_h < 10 or tile_w < 10:
		tile_h, tile_w = h, w

	labeled_tiles = []
	for i, p in enumerate(processed):
		resized = cv2.resize(p, (tile_w, tile_h), interpolation=cv2.INTER_NEAREST)
		color = np.array(palette[i], dtype=np.uint8)
		colored = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
		mask_bool = (resized == 255)
		for c in range(3):
			colored[:, :, c][mask_bool] = color[c]

		# overlay filename (basename without extension)
		name = os.path.splitext(os.path.basename(sem_maps[i]))[0]
		# put a semi-transparent rectangle behind text for readability
		txt = name
		font = cv2.FONT_HERSHEY_SIMPLEX
		scale = max(0.4, min(tile_w, tile_h) / 200.0)
		thickness = 1
		(tw, th), _ = cv2.getTextSize(txt, font, scale, thickness)
		pad = 4
		# rectangle position: bottom-left
		x0, y0 = 5, tile_h - th - 5
		x1, y1 = x0 + tw + pad, y0 + th + pad//2
		# draw filled rectangle (semi-transparent effect by mixing)
		overlay = colored.copy()
		cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), -1)
		cv2.addWeighted(overlay, 0.5, colored, 0.5, 0, colored)
		# put text in white
		cv2.putText(colored, txt, (x0 + 2, y0 + th), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

		labeled_tiles.append(colored)

	row1 = np.hstack(labeled_tiles[0:4])
	row2 = np.hstack(labeled_tiles[4:8])
	labeled_grid = np.vstack([row1, row2])

	# show only labeled grid (do not show combined_overlay)
	cv2.namedWindow('semantics_grid', cv2.WINDOW_NORMAL)
	cv2.imshow('semantics_grid', labeled_grid)

	# Determine which semantics are within valid mask area (non-zero pixels)
	areas = [int(np.count_nonzero(p == 255)) for p in processed]
	# indices with area > 0
	valid_idx = [i for i, a in enumerate(areas) if a > 0]
	if len(valid_idx) == 0:
		print('No valid semantics after masking.')
	else:
		# if more than 2 valid, pick two largest by area
		if len(valid_idx) > 2:
			sorted_idx = sorted(valid_idx, key=lambda i: areas[i], reverse=True)
			chosen = sorted_idx[:2]
		else:
			chosen = valid_idx

		# Build a popup image showing the chosen semantics side-by-side with labels
		popup_tiles = []
		for i in chosen:
			p = processed[i]
			color = np.array(palette[i], dtype=np.uint8)
			colored = np.zeros((h, w, 3), dtype=np.uint8)
			mask_bool = (p == 255)
			for c in range(3):
				colored[:, :, c][mask_bool] = color[c]
			# overlay filename and area
			name = os.path.splitext(os.path.basename(sem_maps[i]))[0]
			txt = f'{name} - area: {areas[i]}'
			font = cv2.FONT_HERSHEY_SIMPLEX
			scale = max(0.6, min(w, h) / 400.0)
			thickness = 2
			(tw, th), _ = cv2.getTextSize(txt, font, scale, thickness)
			cv2.rectangle(colored, (5, 5), (5 + tw + 8, 5 + th + 8), (0, 0, 0), -1)
			cv2.putText(colored, txt, (9, 5 + th), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
			popup_tiles.append(colored)

		# if only one chosen, duplicate second for layout
		if len(popup_tiles) == 1:
			popup_tiles.append(popup_tiles[0].copy())

	popup = np.hstack(popup_tiles)
	cv2.namedWindow('top_semantics', cv2.WINDOW_NORMAL)
	cv2.imshow('top_semantics', popup)

	def compare_and_popup(processed_list: List[np.ndarray]) -> Optional[bool]:
		"""Build popup for chosen semantics from processed_list and compare numeric labels.
		Returns True if upper > lower, False if upper <= lower, None if cannot decide."""
		areas_local = [int(np.count_nonzero(p == 255)) for p in processed_list]
		valid_idx_local = [i for i, a in enumerate(areas_local) if a > 0]
		if len(valid_idx_local) == 0:
			print('No valid semantics after masking for this mask.')
			return None

		if len(valid_idx_local) > 2:
			sorted_idx_local = sorted(valid_idx_local, key=lambda i: areas_local[i], reverse=True)
			chosen_local = sorted_idx_local[:2]
		else:
			chosen_local = valid_idx_local

		# build popup tiles for chosen_local
		popup_tiles_local = []
		for i in chosen_local:
			p = processed_list[i]
			color = np.array(palette[i], dtype=np.uint8)
			colored = np.zeros((h, w, 3), dtype=np.uint8)
			mask_bool = (p == 255)
			for c in range(3):
				colored[:, :, c][mask_bool] = color[c]
			name = os.path.splitext(os.path.basename(sem_maps[i]))[0]
			txt = f'{name} - area: {areas_local[i]}'
			font = cv2.FONT_HERSHEY_SIMPLEX
			scale = max(0.6, min(w, h) / 400.0)
			thickness = 2
			(tw, th), _ = cv2.getTextSize(txt, font, scale, thickness)
			cv2.rectangle(colored, (5, 5), (5 + tw + 8, 5 + th + 8), (0, 0, 0), -1)
			cv2.putText(colored, txt, (9, 5 + th), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
			popup_tiles_local.append(colored)

		if len(popup_tiles_local) == 1:
			popup_tiles_local.append(popup_tiles_local[0].copy())

		popup_local = np.hstack(popup_tiles_local)
		cv2.namedWindow('top_semantics_local', cv2.WINDOW_NORMAL)
		cv2.imshow('top_semantics_local', popup_local)

		# compute centroids and numeric vals
		centroids_local = []
		numeric_vals_local = []
		import re
		for i in chosen_local:
			p = processed_list[i]
			ys, xs = np.where(p == 255)
			if len(xs) == 0:
				cx, cy = (0.0, 0.0)
			else:
				cx = float(xs.mean())
				cy = float(ys.mean())
			centroids_local.append((cx, cy))
			name = os.path.splitext(os.path.basename(sem_maps[i]))[0]
			m = re.search(r"(-?\d+)", name)
			if m:
				try:
					val = int(m.group(1))
				except Exception:
					val = None
			else:
				val = None
			numeric_vals_local.append(val)

		if len(centroids_local) >= 2:
			y0 = centroids_local[0][1]
			y1 = centroids_local[1][1]
			if y0 < y1:
				upper_idx = 0
				lower_idx = 1
			else:
				upper_idx = 1
				lower_idx = 0

			upper_val = numeric_vals_local[upper_idx]
			lower_val = numeric_vals_local[lower_idx]

			if upper_val is None or lower_val is None:
				print('Warning: could not parse numeric labels from filenames:', [os.path.basename(sem_maps[i]) for i in chosen_local])
				# fallback: attempt to parse full name as float
				try:
					u = float(os.path.splitext(os.path.basename(sem_maps[chosen_local[upper_idx]]))[0])
					l = float(os.path.splitext(os.path.basename(sem_maps[chosen_local[lower_idx]]))[0])
					upper_val = u
					lower_val = l
				except Exception:
					print('Cannot determine numeric comparison for chosen_local')
					return None

			# both exist now
			return upper_val > lower_val
		else:
			# only one chosen
			print('Only one semantic present; cannot compare. No output.')
			return None

	# perform first stage decision only if start_state == 0
	if start_state == 0:
		first_cmp = compare_and_popup(processed)
		if first_cmp is True:
			state = 1
		elif first_cmp is False:
			state = 2
		else:
			state = 2  # default fallback
		print('state after stage1:', state)

		# Stage 2: use mask2 and decide final state 3 or 4
		combined2, processed2 = run_with_mask('mask2.jpg')
	else:
		# start_state is 1 or 2: skip stage1 and use mask2 directly
		combined2, processed2 = run_with_mask('mask2.jpg')

	second_cmp = compare_and_popup(processed2)
	if second_cmp is True:
		state = 3
	elif second_cmp is False:
		state = 4
	else:
		state = 4
	print('state after stage2:', state)

	print('Press any key on the image window to continue...')
	cv2.waitKey(0)
	cv2.destroyAllWindows()

	print('Done. combined shape:', combined.shape)

