#!/usr/bin/env python3
"""
将 RGB 中的 YOLO 格式 bbox 投影到 camera_link 的 xOy 平面并在点云中可视化。
用法示例：
    python3 project_box_to_ground.py --ws . --rgb-file rgb/1305031102.160407.png --bbox rgb/1.txt --save

行为：
- 读取 camera_para.txt 获取 fx,fy,cx,cy,factor
- 读取指定 rgb 图片和对应深度图（按相同文件名在 depth/ 下查找）
- 读取 bbox（YOLO 格式： class cx_norm cy_norm w_norm h_norm）
- 将 bbox 区域的深度反投影为 3D 点云（相机坐标系，单位 m），再将这些点垂直投影到 Z=0 平面（保持 x,y）
- 在 Open3D 中显示原始点云（灰色）和投影点（红色），若 --save 则保存为 bbox_projected.ply
"""
#!/usr/bin/env python3
"""
将 RGB 中的 YOLO 格式 bbox 投影到 camera_link 的 xOy 平面并在点云中可视化。
用法示例：
  python3 project_box_to_ground.py --ws . --rgb-file rgb/1305031102.160407.png --bbox rgb/1.txt --save

行为：
- 读取 camera_para.txt 获取 fx,fy,cx,cy,factor
- 读取指定 rgb 图片和对应深度图（按相同文件名在 depth/ 下查找）
- 读取 bbox（YOLO 格式： class cx_norm cy_norm w_norm h_norm）
- 将 bbox 区域的深度反投影为 3D 点云（相机坐标系，单位 m），再将这些点垂直投影到 Z=0 平面（保持 x,y）
- 在 Open3D 中显示原始点云（灰色）和投影点（红色），若 --save 则保存为 bbox_projected.ply
"""

import argparse
import os
from pathlib import Path
import numpy as np
import open3d as o3d
import cv2
import config as cfg


def parse_camera_para(path):
    fx = fy = cx = cy = None
    factor = 1.0
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or line == '':
                continue
            if '=' in line:
                parts = line.split('#')[0].split('=')
                if len(parts) >= 2:
                    key = parts[0].strip()
                    try:
                        val = float(parts[1].strip())
                    except:
                        continue
                    if key == 'fx':
                        fx = val
                    elif key == 'fy':
                        fy = val
                    elif key == 'cx':
                        cx = val
                    elif key == 'cy':
                        cy = val
                    elif key == 'factor':
                        factor = val
    if None in (fx, fy, cx, cy):
        raise RuntimeError(f'无法在 {path} 中解析相机内参 fx/fy/cx/cy')
    return float(fx), float(fy), float(cx), float(cy), float(factor)


def yolo_to_bbox_pixels(yolo_line, img_w, img_h):
    # yolo_line like: class cx_norm cy_norm w_norm h_norm
    parts = yolo_line.strip().split()
    if len(parts) < 5:
        raise RuntimeError('YOLO 框格式不对，需: class cx cy w h')
    cls = int(float(parts[0]))
    cxn = float(parts[1])
    cyn = float(parts[2])
    wn = float(parts[3])
    hn = float(parts[4])
    cx = cxn * img_w
    cy = cyn * img_h
    w = wn * img_w
    h = hn * img_h
    x1 = int(round(cx - w/2))
    y1 = int(round(cy - h/2))
    x2 = int(round(cx + w/2))
    y2 = int(round(cy + h/2))
    # clamp
    x1 = max(0, min(img_w-1, x1))
    x2 = max(0, min(img_w-1, x2))
    y1 = max(0, min(img_h-1, y1))
    y2 = max(0, min(img_h-1, y2))
    return cls, x1, y1, x2, y2


def depth_region_to_points(depth_img, x1, y1, x2, y2, fx, fy, cx, cy, factor, depth_trunc=3.0):
    # depth_img: numpy array, raw as read from OpenCV (uint16 or float)
    h, w = depth_img.shape
    xs = []
    ys = []
    zs = []
    colors = []
    # helper: average 3x3 neighborhood raw depth (ignore zeros and out-of-range after scaling)
    def local_avg_raw_depth(u, v):
        vals = []
        for dv in (-1, 0, 1):
            for du in (-1, 0, 1):
                uu = u + du
                vv = v + dv
                if uu < 0 or uu >= w or vv < 0 or vv >= h:
                    continue
                z_r = depth_img[vv, uu]
                if z_r == 0:
                    continue
                # convert to meters and check truncation
                z_m = float(z_r) / float(factor)
                if z_m <= 0 or z_m > depth_trunc:
                    continue
                vals.append(float(z_r))
        if len(vals) == 0:
            return 0
        return sum(vals) / len(vals)

    for v in range(y1, y2+1):
        for u in range(x1, x2+1):
            if v < 0 or v >= h or u < 0 or u >= w:
                continue
            if cfg.USE_3x3_DEPTH_AVG:
                z_raw = local_avg_raw_depth(u, v)
            else:
                z_raw = depth_img[v, u]
            if z_raw == 0:
                continue
            z = float(z_raw) / float(factor)
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            xs.append(x)
            ys.append(y)
            zs.append(z)
    if len(xs) == 0:
        return np.zeros((0,3))
    pts = np.stack([xs, ys, zs], axis=1)
    return pts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save', action='store_true', help='save output PLY to ws/bbox_projected.ply')
    parser.add_argument('--depth-trunc', type=float, default=cfg.DEPTH_TRUNC, help='max depth to keep (m)')
    args = parser.parse_args()
    # 使用 config 中的路径和参数
    ws = cfg.WS
    cam_file = ws / 'camera_para.txt'
    if cam_file.exists():
        fx, fy, cx, cy, factor = parse_camera_para(str(cam_file))
        print('从 camera_para.txt 读取相机内参:', fx, fy, cx, cy, 'factor=', factor)
    else:
        cam = cfg.CAMERA
        fx, fy, cx, cy, factor = cam['fx'], cam['fy'], cam['cx'], cam['cy'], cam['factor']
        print('使用 config.py 中的相机内参:', fx, fy, cx, cy, 'factor=', factor)

    # 内置 RGB 与 bbox 路径（从 config 中读取）
    rgb_rel = cfg.RGB_FILE
    bbox_rel = cfg.BBOX_FILE

    rgb_path = ws / rgb_rel
    if not rgb_path.exists():
        print('找不到 rgb 图像:', rgb_path)
        return
    depth_path = ws / 'depth' / rgb_path.name
    if not depth_path.exists():
        print('找不到对应深度图 (期待相同文件名在 depth/):', depth_path)
        return

    color = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    if color is None or depth is None:
        print('读取图像失败')
        return
    img_h, img_w = color.shape[0], color.shape[1]

    # 读取 bbox 文件，支持多行（逐个处理），但这里仅使用第一行
    bbox_file = ws / bbox_rel
    if not bbox_file.exists():
        print('找不到 bbox 文件:', bbox_file)
        return
    with open(bbox_file, 'r') as f:
        lines = [l.strip() for l in f.readlines() if l.strip()!='']
    if len(lines) == 0:
        print('bbox 文件为空')
        return
    cls, x1, y1, x2, y2 = yolo_to_bbox_pixels(lines[0], img_w, img_h)
    print('bbox pixels:', x1, y1, x2, y2)

    # 生成整幅点云以供显示
    color_o3d = o3d.geometry.Image(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
    depth_o3d = o3d.geometry.Image(depth)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d, depth_scale=factor, depth_trunc=args.depth_trunc, convert_rgb_to_intensity=False)

    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width=img_w, height=img_h, fx=fx, fy=fy, cx=cx, cy=cy)
    full_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    # same flip as rgbd_to_ply for natural visualization
    flip_transform = np.array([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    full_pcd.transform(flip_transform)

    # helper: rpy -> rotation and 4x4 transform builder (used for map<->camera transforms)
    def rpy_to_rot(roll, pitch, yaw):
        Rx = np.array([[1,0,0],[0,np.cos(roll), -np.sin(roll)],[0,np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw),0],[np.sin(yaw), np.cos(yaw),0],[0,0,1]])
        return Rz @ Ry @ Rx

    def build_transform(tx, ty, tz, roll, pitch, yaw):
        R = rpy_to_rot(roll, pitch, yaw)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.array([tx, ty, tz])
        return T

    # 从 bbox 区域提取点
    pts = depth_region_to_points(depth, x1, y1, x2, y2, fx, fy, cx, cy, factor, depth_trunc=args.depth_trunc)
    if pts.shape[0] == 0:
        print('在 bbox 区域没有找到有效深度点')
        return

    # 投影到 xOy 平面：使用 STATIC_TRANSFORM（始终）
    st = cfg.STATIC_TRANSFORM
    T_map2cam = build_transform(st['tx'], st['ty'], st['tz'], st['roll'], st['pitch'], st['yaw'])
    T_cam2map = np.linalg.inv(T_map2cam)
    # pts are in camera coords; transform to map coords
    homo_pts = np.concatenate([pts, np.ones((pts.shape[0],1))], axis=1)
    pts_in_map_h = (T_cam2map @ homo_pts.T).T
    pts_in_map = pts_in_map_h[:, :3]
    # set z=0 in map frame (project to map z=0)
    proj_map = pts_in_map.copy()
    proj_map[:, 2] = 0.0
    # transform projected points back to camera frame
    homo_proj_map = np.concatenate([proj_map, np.ones((proj_map.shape[0],1))], axis=1)
    proj_cam_h = (T_map2cam @ homo_proj_map.T).T
    proj_pts = proj_cam_h[:, :3]

    # 将 obj_pts 和 proj_pts 变换到与 full_pcd 相同的显示坐标系
    def apply_transform(pts, T):
        if pts.shape[0] == 0:
            return pts
        homo = np.concatenate([pts, np.ones((pts.shape[0],1))], axis=1)
        trans = (T @ homo.T).T
        return trans[:, :3]

    pts_t = apply_transform(pts, flip_transform)
    proj_pts_t = apply_transform(proj_pts, flip_transform)

    # 构造 Open3D 点云对象用于可视化: 原点云 + 投影点
    obj_pcd = o3d.geometry.PointCloud()
    obj_pcd.points = o3d.utility.Vector3dVector(pts_t)
    # color red
    obj_pcd.colors = o3d.utility.Vector3dVector(np.tile(np.array([cfg.COLOR_OBJ]), (pts.shape[0],1)))

    proj_pcd = o3d.geometry.PointCloud()
    proj_pcd.points = o3d.utility.Vector3dVector(proj_pts_t)
    proj_pcd.colors = o3d.utility.Vector3dVector(np.tile(np.array([cfg.COLOR_PROJ]), (proj_pts.shape[0],1)))

    # 构造一个表示 xOy 平面的点云（在 z=0 平面上采样），范围根据 full_pcd 的 xy 边界
    # 为了让平面可由 map 和 STATIC_TRANSFORM 决定，我们在两种模式下绘制平面：
    # - 如果 cfg.USE_COMPUTED_MAP_ORIGIN 为 True：仍然在相机坐标系的 xy 边界上采样（原有行为）
    # - 否则：在 map 坐标系的 z=0 平面上采样（网格范围基于投影点在 map 中的范围），然后把这些点变换到相机坐标系用于显示

    def rpy_to_rot(roll, pitch, yaw):
        # rotation matrix from roll (x), pitch (y), yaw (z)
        Rx = np.array([[1,0,0],[0,np.cos(roll), -np.sin(roll)],[0,np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw),0],[np.sin(yaw), np.cos(yaw),0],[0,0,1]])
        # ROS convention roll,pitch,yaw -> R = Rz * Ry * Rx
        return Rz @ Ry @ Rx

    def build_transform(tx, ty, tz, roll, pitch, yaw):
        R = rpy_to_rot(roll, pitch, yaw)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.array([tx, ty, tz])
        return T

    # 使用 STATIC_TRANSFORM 定义 map->camera 的变换：在 map 的 z=0 平面上采样，然后变换到相机坐标系
    st = cfg.STATIC_TRANSFORM
    T_map2cam = build_transform(st['tx'], st['ty'], st['tz'], st['roll'], st['pitch'], st['yaw'])
    # 将投影点 proj_pts (相机坐标系) 转到 map 坐标系，以确定采样范围
    try:
        T_cam2map = np.linalg.inv(T_map2cam)
        # proj_pts are in camera coords (before flip)
        homo = np.concatenate([proj_pts, np.ones((proj_pts.shape[0],1))], axis=1)
        proj_in_map_h = (T_cam2map @ homo.T).T
        proj_in_map = proj_in_map_h[:, :3]
        if proj_in_map.shape[0] > 0:
            min_xy = proj_in_map[:, :2].min(axis=0)
            max_xy = proj_in_map[:, :2].max(axis=0)
        else:
            min_xy = np.array([-1.0, -1.0])
            max_xy = np.array([1.0, 1.0])
    except Exception:
        min_xy = np.array([-1.0, -1.0])
        max_xy = np.array([1.0, 1.0])

    pad = cfg.PLANE_PADDING
    x_min, y_min = min_xy - pad
    x_max, y_max = max_xy + pad
    grid_res = cfg.PLANE_SAMPLING_RES

    # ROI 区域采样（来自 config.py）
    roi_x_min, roi_y_min = cfg.MAP_ROI_MIN
    roi_x_max, roi_y_max = cfg.MAP_ROI_MAX
    xs_roi = np.arange(roi_x_min, roi_x_max, grid_res)
    ys_roi = np.arange(roi_y_min, roi_y_max, grid_res)
    gx_roi, gy_roi = np.meshgrid(xs_roi, ys_roi)
    gz_roi = np.zeros_like(gx_roi)
    plane_pts_map_roi = np.stack([gx_roi.ravel(), gy_roi.ravel(), gz_roi.ravel()], axis=1)
    # 将 ROI 区域 map 平面点变换到相机坐标系
    homo_map_roi = np.concatenate([plane_pts_map_roi, np.ones((plane_pts_map_roi.shape[0],1))], axis=1)
    plane_cam_h_roi = (T_map2cam @ homo_map_roi.T).T
    plane_pts_cam_roi = plane_cam_h_roi[:, :3]
    plane_pts_cam_roi_t = apply_transform(plane_pts_cam_roi, flip_transform)
    roi_plane_pcd = o3d.geometry.PointCloud()
    roi_plane_pcd.points = o3d.utility.Vector3dVector(plane_pts_cam_roi_t)
    roi_plane_pcd.colors = o3d.utility.Vector3dVector(np.tile(np.array([cfg.COLOR_PLANE]), (plane_pts_cam_roi_t.shape[0],1)))

    # 生成 ROI 区域坐标轴（以 ROI 区域中心为原点），在 roi_plane_pcd 创建之后确保依赖已定义
    roi_x_min, roi_y_min = cfg.MAP_ROI_MIN
    roi_x_max, roi_y_max = cfg.MAP_ROI_MAX
    axis_origin = np.array([(roi_x_min + roi_x_max)/2, (roi_y_min + roi_y_max)/2, 0.0])
    axis_len = 1.0  # 坐标轴长度（米）
    axis_pts_map = np.array([
        axis_origin,
        axis_origin + [axis_len,0,0],
        axis_origin,
        axis_origin + [0,axis_len,0],
        axis_origin,
        axis_origin + [0,0,axis_len],
    ])
    homo_axis = np.concatenate([axis_pts_map, np.ones((axis_pts_map.shape[0],1))], axis=1)
    axis_cam_h = (T_map2cam @ homo_axis.T).T
    axis_cam = axis_cam_h[:, :3]
    axis_cam_t = apply_transform(axis_cam, flip_transform)
    axis_lines = [[0,1],[2,3],[4,5]]
    axis_colors = [[1,0,0],[0,1,0],[0,0,1]]
    axis_set = o3d.geometry.LineSet()
    axis_set.points = o3d.utility.Vector3dVector(axis_cam_t)
    axis_set.lines = o3d.utility.Vector2iVector(axis_lines)
    axis_set.colors = o3d.utility.Vector3dVector(axis_colors)

    # 原有全局平面采样（用于主窗口）
    xs = np.arange(x_min, x_max, grid_res)
    ys = np.arange(y_min, y_max, grid_res)
    gx, gy = np.meshgrid(xs, ys)
    gz = np.zeros_like(gx)
    plane_pts_map = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    homo_map = np.concatenate([plane_pts_map, np.ones((plane_pts_map.shape[0],1))], axis=1)
    plane_cam_h = (T_map2cam @ homo_map.T).T
    plane_pts_cam = plane_cam_h[:, :3]
    plane_pts_cam_t = apply_transform(plane_pts_cam, flip_transform)
    plane_pcd = o3d.geometry.PointCloud()
    plane_pcd.points = o3d.utility.Vector3dVector(plane_pts_cam_t)
    plane_pcd.colors = o3d.utility.Vector3dVector(np.tile(np.array([cfg.COLOR_PLANE]), (plane_pts_cam_t.shape[0],1)))

    # 输出绿色投影点在 map 平面上的中心坐标
    if proj_pts.shape[0] > 0:
        homo_proj = np.concatenate([proj_pts, np.ones((proj_pts.shape[0],1))], axis=1)
        proj_in_map_h = (T_cam2map @ homo_proj.T).T
        proj_in_map = proj_in_map_h[:, :3]
        center_map = proj_in_map.mean(axis=0)
        center_map[2] = 0.0
        print(f'绿色投影点在 map 平面上的中心坐标: (x={center_map[0]:.3f}, y={center_map[1]:.3f}, z=0.000)')

    # 生成 ROI 区域内的绿色投影点
    if proj_pts.shape[0] > 0:
        # proj_pts: camera系，先变到map系
        homo_proj = np.concatenate([proj_pts, np.ones((proj_pts.shape[0],1))], axis=1)
        proj_in_map_h = (T_cam2map @ homo_proj.T).T
        proj_in_map = proj_in_map_h[:, :3]
        # 筛选在 ROI 区域内的点
        roi_x_min, roi_y_min = cfg.MAP_ROI_MIN
        roi_x_max, roi_y_max = cfg.MAP_ROI_MAX
        mask = (proj_in_map[:,0] >= roi_x_min) & (proj_in_map[:,0] <= roi_x_max) & (proj_in_map[:,1] >= roi_y_min) & (proj_in_map[:,1] <= roi_y_max)
        roi_proj_map = proj_in_map[mask]
        # 再变回 camera 系
        if roi_proj_map.shape[0] > 0:
            roi_proj_map[:,2] = 0.0
            homo_roi_proj_map = np.concatenate([roi_proj_map, np.ones((roi_proj_map.shape[0],1))], axis=1)
            roi_proj_cam_h = (T_map2cam @ homo_roi_proj_map.T).T
            roi_proj_cam = roi_proj_cam_h[:, :3]
            roi_proj_cam_t = apply_transform(roi_proj_cam, flip_transform)
            roi_proj_pcd = o3d.geometry.PointCloud()
            roi_proj_pcd.points = o3d.utility.Vector3dVector(roi_proj_cam_t)
            roi_proj_pcd.colors = o3d.utility.Vector3dVector(np.tile(np.array([cfg.COLOR_PROJ]), (roi_proj_cam_t.shape[0],1)))
            o3d.visualization.draw_geometries([roi_plane_pcd, roi_proj_pcd, axis_set], window_name='Map ROI 平面区域')
        else:
            o3d.visualization.draw_geometries([roi_plane_pcd, axis_set], window_name='Map ROI 平面区域')
    else:
        o3d.visualization.draw_geometries([roi_plane_pcd, axis_set], window_name='Map ROI 平面区域')

    # 主弹窗：显示完整点云(保留 RGB) + 物体点(红) + 投影(绿) + 全局平面(蓝)
    display_pcds = []
    display_pcds.append(full_pcd)
    display_pcds.append(obj_pcd)
    display_pcds.append(proj_pcd)
    display_pcds.append(plane_pcd)
    o3d.visualization.draw_geometries(display_pcds, window_name='BBox 投影到 xOy 平面')

    # 保存
    if args.save:
        save_path = ws / cfg.OUTPUT_PLY
        # 合并点云并保存为点云（包括 plane 采样点），保留 full_pcd 的颜色
        merged = full_pcd + obj_pcd + proj_pcd + plane_pcd
        o3d.io.write_point_cloud(str(save_path), merged)
        print('已保存到', save_path)

    # 始终使用 config 中预设的静态变换
    print('\n使用 config.py 中的静态变换：')
    print(cfg.static_transform_cmd_from_static())


if __name__ == '__main__':
    main()
