import cv2
import numpy as np

def segment_blue_road_sign(img):
    """
    使用 HSV 颜色空间对图像中的蓝色目标进行分割。

    Args:
        img (np.ndarray): 待处理的 BGR 图像。

    Returns:
        np.ndarray: 经过二值化分割后的掩膜 (Mask)，蓝色部分为白色 (255)。
    """
    if img is None:
        return None

    # 1. 将图像从 BGR 转换到 HSV 颜色空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 蓝色在 HSV 空间的典型范围
    # H: 100-130, S: 80-255, V: 80-255
    lower_blue = np.array([100, 80, 80])
    upper_blue = np.array([130, 255, 255])

    # 2. 创建掩膜 (Mask)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    return mask_blue

def segment_red_road_sign(img):
    """
    使用 HSV 颜色空间对图像中的纯红色目标进行分割。

    Args:
        img (np.ndarray): 待处理的 BGR 图像。

    Returns:
        np.ndarray: 经过二值化分割后的掩膜 (Mask)，红色部分为白色 (255)。
    """
    if img is None:
        return None

    # 1. 将图像从 BGR 转换到 HSV 颜色空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # # 2. 定义红色在 HSV 空间中的阈值范围
    # # 红色范围 1
    # H：[0, 10]
    # S：[95, 255]
    # V：[150, 255]
    lower_red_1 = np.array([0, 65, 100])
    upper_red_1 = np.array([30, 255, 255])
    
    # # 红色范围 2
    # H：[170, 180]
    # S：[95, 255]
    # V：[150, 255]
    lower_red_2 = np.array([150, 65, 100])
    upper_red_2 = np.array([180, 255, 255])

    # 3. 创建掩膜 (Mask) 并合并
    mask_red_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask_red_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    final_mask = cv2.bitwise_or(mask_red_1, mask_red_2)
    
    return final_mask