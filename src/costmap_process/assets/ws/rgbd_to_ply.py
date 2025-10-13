#!/usr/bin/env python3
"""
生成点云：读取 ./ws/camera_para.txt、./ws/rgb/ 和 ./ws/depth/ 下的图像，保存为 PLY。
默认会选取各自文件夹中第一个按字典序排序的图像对。
依赖：open3d, opencv-python, numpy
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import cv2


def parse_camera_para(path):
    fx = fy = cx = cy = None
    factor = 1.0
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or line == '':
                continue
            if '=' in line:
                # try to parse like "fx = 525.0"
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


def find_first_image_pair(rgb_dir, depth_dir):
    rgb_files = sorted([p for p in os.listdir(rgb_dir) if not p.startswith('.')])
    depth_files = sorted([p for p in os.listdir(depth_dir) if not p.startswith('.')])
    if len(rgb_files) == 0 or len(depth_files) == 0:
        raise RuntimeError('rgb 或 depth 目录为空')
    # try to find file with same stem, otherwise use first
    for r in rgb_files:
        stem = Path(r).stem
        for d in depth_files:
            if Path(d).stem == stem:
                return os.path.join(rgb_dir, r), os.path.join(depth_dir, d)
    return os.path.join(rgb_dir, rgb_files[0]), os.path.join(depth_dir, depth_files[0])


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--ws', default='.', help='workspace path (contains camera_para.txt, rgb/, depth/)')
    p.add_argument('--rgb', default='rgb', help='rgb subfolder name')
    p.add_argument('--depth', default='depth', help='depth subfolder name')
    p.add_argument('--camera', default='camera_para.txt', help='camera parameter file name')
    p.add_argument('--out', default='output.ply', help='output PLY filename')
    p.add_argument('--no-visualize', dest='visualize', action='store_false', help='do not open visualizer')
    p.add_argument('--depth-trunc', type=float, default=3.0, help='max depth (m) for truncation')
    args = p.parse_args()

    ws = Path(args.ws)
    cam_file = ws / args.camera
    rgb_dir = ws / args.rgb
    depth_dir = ws / args.depth

    if not cam_file.exists():
        print(f'找不到相机参数文件: {cam_file}', file=sys.stderr)
        sys.exit(2)
    if not rgb_dir.exists() or not depth_dir.exists():
        print(f'找不到 rgb/ 或 depth/ 目录: {rgb_dir}, {depth_dir}', file=sys.stderr)
        sys.exit(2)

    fx, fy, cx, cy, factor = parse_camera_para(str(cam_file))
    print('相机内参:', fx, fy, cx, cy, 'factor=', factor)

    rgb_path, depth_path = find_first_image_pair(str(rgb_dir), str(depth_dir))
    print('使用图像对:', rgb_path, depth_path)

    color_raw = o3d.io.read_image(rgb_path)
    depth_raw = o3d.io.read_image(depth_path)

    # create rgbd image. open3d expects depth in the integer image where depth/ depth_scale = meters
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw, depth_scale=factor, depth_trunc=args.depth_trunc, convert_rgb_to_intensity=False
    )

    # get image size from color
    w = np.asarray(color_raw).shape[1]
    h = np.asarray(color_raw).shape[0]

    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width=w, height=h, fx=fx, fy=fy, cx=cx, cy=cy)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    # Open3D uses camera forward = +z, but images often need flipping for proper visualization
    # Flip it so that the point cloud looks 'natural' when visualized
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    out_path = ws / args.out
    o3d.io.write_point_cloud(str(out_path), pcd)
    print('已保存点云到:', out_path)

    if args.visualize:
        o3d.visualization.draw_geometries([pcd], window_name='RGB-D 点云')


if __name__ == '__main__':
    main()
