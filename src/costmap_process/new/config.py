# 全局配置文件（ws/config.py）
# 在这里集中管理路径、相机内参、深度参数和 TF 静态变换的默认值。
# 请根据需要修改这些值，脚本（例如 project_box_to_ground.py）可以 import 这个模块来使用统一配置。

from pathlib import Path

# 相机内参（可覆盖 camera_para.txt 中的值或用于没有文件时的默认值）
# 单位：fx,fy 像素；cx,cy 像素；factor 为深度缩放（depth_raw / factor = meters）
CAMERA = {
    'fx': 543.73,
    'fy': 543.73,
    'cx': 318.34,
    'cy': 247.11,
    'factor': 1.0,
}
# map 平面 ROI 区域（单位米，左下角和右上角）
MAP_ROI_MIN = (0, 0)  # (x_min, y_min)
MAP_ROI_MAX = (5, 3)  # (x_max, y_max)

# 静态变换（如果不使用动态计算质心，可以在这里给出 map -> camera_init 的固定变换）
# 格式为：tx, ty, tz, roll, pitch, yaw
# 示例命令：ros2 run tf2_ros static_transform_publisher tx ty tz roll pitch yaw map camera_init
STATIC_TRANSFORM = {
    'tx': 4.6,
    'ty': 0.77,
    'tz': 0.1, # previous 0.25
    'roll': 3.1415926,
    'pitch': 0.0,
    'yaw': 0.0,
    'parent_frame': 'map',
    'child_frame': 'camera_init',
}
def static_transform_cmd_from_static():
    t = STATIC_TRANSFORM
    return f"ros2 run tf2_ros static_transform_publisher {t['tx']} {t['ty']} {t['tz']} {t['roll']} {t['pitch']} {t['yaw']} {t['parent_frame']} {t['child_frame']}"

# 用一个先验信息去，用于推导真实 xz
CAMERA_PRIORI = {
    'v_0': 314,
    'X_0': 0.8,
    'height': 0.11
}

# 可选的 origin map 路径常量，供 draw_map 等模块使用。若你在其他位置使用不同路径，可覆盖此值。
# 默认为项目内的示例地图文件（若不存在，请根据你的工作区调整路径）。
MAP_ORIGIN_PATH = '/Pav-S_ws/src/costmap_process/assets/map/map_origin.jpg'

# 显示origin map上绘制内容的弹窗
SHOW_ORIGIN_MAP_WINDOW = False
LOCAL = False

# Occupancy map / publish settings
# 默认发布 topic 名称
OCCUPANCY_TOPIC = 'origin_map_occupancy'
# 默认分辨率（若为 None 则从 MAP_ROI 与图片尺寸计算）
DEFAULT_OCCUPANCY_RESOLUTION = None
# 颜色映射与占用值（方案 A：简洁高对比）
# key 可以是类别名（如 'red_zone'）或字符串化的数字 '1'..'8'
OCCUPANCY_COLOR_MAP = {
    # 注：rgb 为 (R, G, B) 三元组。下面注释给出常见颜色名称（中文/英文）便于识别。
    'red_zone':   {'rgb': (220, 20, 60),  'occ': 100, 'priority': 1},   # 深红 / Crimson
    'red_cone':   {'rgb': (255, 69, 0),   'occ': 100, 'priority': 2},   # 橙红 / OrangeRed
    'yellow_zone':{'rgb': (255, 215, 0),  'occ': 80,  'priority': 3},   # 金黄 / Gold
    '1':          {'rgb': (31, 119, 180), 'occ': 60,  'priority': 4},   # 蓝色 / Blue (tableau-like)
    '2':          {'rgb': (44, 160, 44),  'occ': 60,  'priority': 5},   # 绿色 / Green
    '3':          {'rgb': (214, 39, 40),  'occ': 60,  'priority': 6},   # 纯红 / Red
    '4':          {'rgb': (148, 103, 189),'occ': 60,  'priority': 7},   # 紫色 / Purple
    '5':          {'rgb': (140, 86, 75),  'occ': 60,  'priority': 8},   # 棕色 / Brown
    '6':          {'rgb': (227, 119, 194),'occ': 60,  'priority': 9},   # 粉色 / Pink
    '7':          {'rgb': (127, 127, 127),'occ': 60,  'priority': 10},  # 灰色 / Gray
    '8':          {'rgb': (188, 189, 34), 'occ': 60,  'priority': 11},  # 橄榄/黄绿色 / Olive
}
# 颜色匹配阈值（RGB 距离）——用于容忍绘图中的抗锯齿等微小差异
OCCUPANCY_COLOR_THRESHOLD = 12
