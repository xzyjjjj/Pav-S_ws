# 全局配置文件（ws/config.py）
# 在这里集中管理路径、相机内参、深度参数和 TF 静态变换的默认值。
# 请根据需要修改这些值，脚本（例如 project_box_to_ground.py）可以 import 这个模块来使用统一配置。

from pathlib import Path

# 工作空间根目录（相对于运行脚本的当前工作目录）
WS = Path('.')

# 数据子目录与文件（相对于 WS）
RGB_DIR = Path('rgb')
DEPTH_DIR = Path('depth')
# 默认 RGB 文件与 bbox 文件（可改为你常用的文件名）
RGB_FILE = RGB_DIR / '1.png'
BBOX_FILE = RGB_DIR / '1.txt'

# 输出
OUTPUT_PLY = Path('bbox_projected.ply')

# 相机内参（可覆盖 camera_para.txt 中的值或用于没有文件时的默认值）
# 单位：fx,fy 像素；cx,cy 像素；factor 为深度缩放（depth_raw / factor = meters）
CAMERA = {
    'fx': 543.73,
    'fy': 543.73,
    'cx': 318.34,
    'cy': 247.11,
    'factor': 1.0,
}

# 深度截断（米）
DEPTH_TRUNC = 3.0

 # 平面采样参数（用于可视化的 xOy 平面）
PLANE_SAMPLING_RES = 0.05  # 米，网格分辨率
PLANE_PADDING = 0.5  # 米，边界扩展

# map 平面 ROI 区域（单位米，左下角和右上角）
MAP_ROI_MIN = (0, 0)  # (x_min, y_min)
MAP_ROI_MAX = (5, 3)  # (x_max, y_max)

# 是否在运行时以投影点集合的质心作为 map 原点（如果 False，将使用下面的 STATIC_TRANSFORM）
USE_COMPUTED_MAP_ORIGIN = False

# 静态变换（如果不使用动态计算质心，可以在这里给出 map -> camera_init 的固定变换）
# 格式为：tx, ty, tz, roll, pitch, yaw
# 示例命令：ros2 run tf2_ros static_transform_publisher tx ty tz roll pitch yaw map camera_init
STATIC_TRANSFORM = {
    'tx': 4.6,
    'ty': 0.77,
    'tz': 0.25,
    'roll': 3.1415926,
    'pitch': 0.0,
    'yaw': 0.0,
    'parent_frame': 'map',
    'child_frame': 'camera_init',
}

# 可视化颜色（RGB 0..1）
COLOR_FULL_PCD = (0.7, 0.7, 0.7)
COLOR_OBJ = (1.0, 0.0, 0.0)
COLOR_PROJ = (0.0, 1.0, 0.0)
COLOR_PLANE = (0.2, 0.4, 1.0)

# 其他可调节项
USE_3x3_DEPTH_AVG = True  # 若 True 则在反投影时对每像素使用 3x3 邻域平均深度

# 便捷函数：将 STATIC_TRANSFORM 输出为 static_transform_publisher 命令字符串
def static_transform_cmd_from_static():
    t = STATIC_TRANSFORM
    return f"ros2 run tf2_ros static_transform_publisher {t['tx']} {t['ty']} {t['tz']} {t['roll']} {t['pitch']} {t['yaw']} {t['parent_frame']} {t['child_frame']}"


if __name__ == '__main__':
    print('WS =', WS)
    print('Default RGB =', RGB_FILE)
    print('Default BBOX =', BBOX_FILE)
    print('Camera intrinsics:', CAMERA)
    print('Static transform cmd (example):')
    print(static_transform_cmd_from_static())
