使用说明

在 `./ws` 目录下运行：

```bash
python3 rgbd_to_ply.py --ws . --out output.ply
```

这会读取 `camera_para.txt`、`rgb/` 与 `depth/` 下的第一个匹配图像对，生成 `output.ply` 并弹出可视化窗口。

依赖安装：

```bash
python3 -m pip install -r requirements.txt
```

注意事项：
- `camera_para.txt` 中应包含 fx, fy, cx, cy 和 factor（depth_scale）。
- 深度图若为 16-bit PNG，factor 典型为 5000（如 dataset 提示）。
- 若你想处理多个帧或生成稠密地图，请使用 Open3D 的 RGB-D Odometry 或 SLAM 工具。