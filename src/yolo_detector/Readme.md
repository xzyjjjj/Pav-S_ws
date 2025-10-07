# YOLO_DETECTOR
## 1. 模块概述
该模块调用提前训练好的 yolov5 模型，对深度相机传入的彩色图片进行目标检测，并发布 \yolo_detections 话题传递检测结果给决策模块。 
其中，detector_module.py 存放封装好的 yolov5 模型类及一些数据处理函数；yolo_node.py 则为封装好ros节点，用于收发消息及调用模型推理。

## 2. 可选参数
启动参数位于/Pav-S_ws/src/yolo_detector/config/params.yaml中，可以修改的参数项包括：  
1. enable_vis：是否开启 opencv 可视化窗口（调试时使用）
2. skip_frames: 跳帧数，每隔多少图片处理一张，可根据算力调整  
3. target_class：默认为''空字符串，可指定为数据集中的某特定类别，则只会检测此类物品（调试时使用）

使用 ros2 launch yolo_detector yolo_detect.launch.py 启动，其中 **模型权重路径** 和 **数据集yaml文件路径** 在该launch文件中指定。

## 3. 启动命令
```bash
ros2 launch yolo_detector yolo_detect.launch.py
```