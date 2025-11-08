publish_costmap.py

说明

这个脚本为 ROS2 (rclpy) 节点：它读取工作目录下的 `costmaps` 文件夹内的四张图片：
- `1_up.jpg` (state=1)
- `1_down.jpg` (state=2)
- `2_up.jpg` (state=3)
- `2_down.jpg` (state=4)

将选中的图片转换为 `nav_msgs/msg/OccupancyGrid` 并发布到话题 `processed_costmap`。

Usage

在包含 ROS2 环境（有 `rclpy`）的机器上：

1. 赋可执行权限：

```bash
chmod +x ~/Desktop/cost/publish_costmap.py
```

2. 运行脚本（示例）：

```bash
~/Desktop/cost/publish_costmap.py --state 1 --rate 1.0
```

参数
- `--state`：选择要发布的图片，1..4（默认 1）
- `--rate`：发布频率（Hz），默认 1.0

映像到占用网格规则（默认实现）
- 假设图片为灰度图：255 (白) -> free -> occupancy=0
- 0 (黑) -> occupied -> occupancy=100
- 阈值为 128
- 分辨率（网格每像素对应的米）在代码中默认设置为 1.0，需要根据实际场景调整：在 `publish_costmap.py` 中修改 `grid.info.resolution`。

注意
- 该脚本假设位于 `~/Desktop/cost/publish_costmap.py`，并且 `costmaps` 文件夹相对于脚本位置存在（`~/Desktop/cost/costmaps`）。
- 如果你的 ROS2 环境使用不同的 frame 或需要不同的 origin/orientation，请在脚本中调整 `grid.info.origin`。

调试模式

如果脚本在非 ROS 环境中运行（没有 `rclpy`），脚本会做一个干运行（dry-run）：加载图片并尝试将其转换为 OccupancyGrid，然后打印尺寸信息，而不实际发布。

rublish_costmap.py 中状态选取的逻辑：
- 通过第一个环岛前
  - 取数字的语义地图，截取第一个环岛前的地图，取出两个数
  - 比较两个数大小，根据大小确定 state
- 之后类似