import cv2
import numpy as np
import os

# ------------------------------------------------
# 1. 定义坐标数据
# 坐标顺序：左上 - 右上 - 右下 - 左下 (x1, y1, x2, y2, x3, y3, x4, y4)
# ------------------------------------------------

src_points_1 = [201.0, 357.0, 442.0, 356.0, 558.0, 447.0, 89.0, 452.0]
src_points_2 = [155.0, 311.0, 464.0, 313.0, 579.0, 358.0, 47.0, 360.0]

# ------------------------------------------------
# 2. 辅助函数：将一维列表转换为 (x, y) 坐标对的列表
# ------------------------------------------------
def list_to_points(point_list):
    """将 [x1, y1, x2, y2, ...] 转换为 [(x1, y1), (x2, y2), ...]"""
    points = []
    for i in range(0, len(point_list), 2):
        points.append((int(point_list[i]), int(point_list[i+1])))
    return points

# ------------------------------------------------
# 3. 图像处理函数（修改为填充黑色区域，其余部分为白色，并移除所有文字输出）
# ------------------------------------------------
def draw_and_save_points(point_lists, output_filename="output_white_black.png"):
    """
    根据给定的坐标点，将矩形区域填充为黑色，图片其余部分为白色，并保存。
    """
    
    # 确定图片尺寸
    # 首先，找到所有坐标中的最大 x 和最大 y 值，以确保图片足够大
    all_coords = [coord for sublist in point_lists for coord in sublist]
    
    # 如果没有提供坐标点，则使用默认尺寸
    if not all_coords:
        max_x, max_y = 800, 600
    else:
        max_x = int(max(all_coords[0::2]) * 1.1)  # 稍微留一点边距 (10%)
        max_y = int(max(all_coords[1::2]) * 1.1)
        # 确保尺寸至少为 1
        max_x = max(max_x, 1)
        max_y = max(max_y, 1)
    
    # 创建一个完全白色的底图 (BGR 格式: 255, 255, 255)
    img = 255 * np.ones((max_y, max_x, 3), dtype=np.uint8)
    
    # 定义填充颜色：黑色 (B=0, G=0, R=0)
    fill_color = (0, 0, 0) 
    
    # 遍历每个坐标点列表，填充黑色矩形
    for point_list in point_lists:
        points = list_to_points(point_list)
        
        # 将点列表转换为 NumPy 数组，这是 cv2.fillPoly 所需的格式
        polygon_points = np.array(points, np.int32).reshape((-1, 1, 2))
        
        # 使用 cv2.fillPoly 填充多边形区域为黑色
        cv2.fillPoly(img, [polygon_points], fill_color)

    # 4. 保存图片
    cv2.imwrite(output_filename, img)
    # 不输出任何文字信息

# ------------------------------------------------
# 4. 运行主函数
# ------------------------------------------------

# 运行代码，生成一个图片，其中矩形区域是黑色的，其余部分是白色的。
draw_and_save_points(
    point_lists=[src_points_1, src_points_2],
    output_filename="output_white_black_rects.png"
)