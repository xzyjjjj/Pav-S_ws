import cv2
import numpy as np

class FilterPoints:
    """ 从 detections 里面读取 types 表，

    用法：
        selector = select_points()
        pts = selector.extract_points(det_json_path, img_path)

    返回值：dict，键为检测索引，值为包含像素 (x,y) 列表的字典：{
        idx: {'bbox': (x_min,y_min,x_max,y_max), 'points': [(x,y), ...], 'crop_rgb': np.ndarray, 'crop_mask': np.ndarray}
    }

    注意：初始化 (__init__) 不读取任何文件。
    """
    def __init__(self):
        # 存放最近一次的掩膜结果（bbox_mask, color_mask, final_mask）
        self.masks = []

    def yolo_mask(self, img, pixel_bbox):
        """
        根据传入的 bbox（四个角点像素坐标）生成二值 mask。
        参数:
            img: numpy.ndarray 原图，用于获取尺寸
            pixel_bbox: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        返回:
            mask: numpy.ndarray uint8 二值掩膜（0/255），shape 与 img 前两维一致。
        """
        if img is None:
            raise ValueError('yolo_mask: img is None')
        h, w = img.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        if pixel_bbox is None or len(pixel_bbox) != 4:
            return mask
        # 取四个角点，计算最小外接矩形
        xs = [int(round(p[0])) for p in pixel_bbox]
        ys = [int(round(p[1])) for p in pixel_bbox]
        x_min = max(0, min(xs))
        x_max = min(w-1, max(xs))
        y_min = max(0, min(ys))
        y_max = min(h-1, max(ys))
        if x_max <= x_min or y_max <= y_min:
            return mask
        mask[y_min:y_max+1, x_min:x_max+1] = 255
        # cv2.imshow('yolo_mask', mask)
        return mask

    def segment_mask(self, img, type):
        """使用颜色分割（red_segment.segment_red_road_sign）或 blue_segment.segment_blue_road_sign 得到掩膜。

        返回 uint8 二值掩膜（0/255）。
        """
        from .color_segment import segment_blue_road_sign, segment_red_road_sign
        if img is None:
            return None
        try:
            if type == 'red_zone':
                mask = segment_red_road_sign(img)
            elif type == 'yellow_zone':
                # yellow 分割暂未实现，返回全零掩膜
                h, w = img.shape[:2]
                mask = np.zeros((h, w), dtype=np.uint8)
            else:
                mask = segment_blue_road_sign(img)
        except Exception:
            # 若导入或调用失败，返回空掩膜
            h, w = img.shape[:2]
            mask = np.zeros((h, w), dtype=np.uint8)
        # cv2.imshow('seg_mask', mask)
        return mask

    def extract_points(self, img, bbox, type):
        """按顺序调用 yolo_mask 与 segment_mask，返回应用掩膜后的图像。

        参数:
            img: BGR 图像
        返回:
            masked_img: 应用最终掩膜后的 BGR 图像（numpy.ndarray）
        """
        if img is None:
            raise ValueError('extract_red_points: img is None')

        # 颜色分割掩膜
        final_mask = self.segment_mask(img, type)
        # bbox 掩膜（若提供）
        bbox_mask = self.yolo_mask(img, bbox)
        if bbox_mask is not None:
            final_mask = cv2.bitwise_and(final_mask, bbox_mask)
        
        # 应用掩膜并返回圖像
        masked_img = cv2.bitwise_and(img, img, mask=final_mask)
        return [masked_img, type]
