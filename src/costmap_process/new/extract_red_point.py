import cv2
import numpy as np

class RedExtracter:
    def filter_small_clusters(self, mask, min_size=500):
        """
        过滤掉点数小于 min_size 的连通域簇，返回过滤后的掩膜。
        """
        import numpy as np
        import cv2
        if mask is None:
            return None
        m = mask
        if m.dtype == np.bool_:
            m = (m.astype(np.uint8) * 255)
        elif m.dtype != np.uint8:
            m = m.astype(np.uint8)
        if m.ndim == 3 and m.shape[2] == 3:
            m = cv2.cvtColor(m, cv2.COLOR_BGR2GRAY)
        bin_mask = (m > 127).astype(np.uint8)
        num_labels, labels = cv2.connectedComponents(bin_mask, connectivity=8)
        filtered_mask = np.zeros_like(bin_mask, dtype=np.uint8)
        for lab in range(1, num_labels):
            ys, xs = np.where(labels == lab)
            if len(xs) >= min_size:
                filtered_mask[labels == lab] = 255
        return filtered_mask
    """
    """
    def __init__(self):
        # 存放最近一次的掩膜结果（bbox_mask, color_mask, final_mask）
        self.masks = []
        self.save_path = '/Pav-S_ws/src/costmap_process/new/debug/'

    def segment_mask(self, img):
        """使用颜色分割（red_segment.segment_red_road_sign）或 blue_segment.segment_blue_road_sign 得到掩膜。

        返回 uint8 二值掩膜（0/255）。
        """
        from .color_segment import segment_red_road_sign
        if img is None:
            return None
        try:
            mask = segment_red_road_sign(img)
        except Exception:
            # 若导入或调用失败，返回空掩膜
            h, w = img.shape[:2]
            mask = np.zeros((h, w), dtype=np.uint8)
        # cv2.imshow('seg_mask', mask)
        return mask

    def save(self, mask):
        if mask is None:
            return
        try:
            from pathlib import Path
            from datetime import datetime
            save_dir = Path(self.save_path)
            save_dir.mkdir(parents=True, exist_ok=True)
            # 确保掩膜为单通道 uint8，值为 0/255
            m = mask
            if m.dtype == np.bool_:
                m = (m.astype(np.uint8) * 255)
            elif m.dtype != np.uint8:
                m = m.astype(np.uint8)
            if m.ndim == 3 and m.shape[2] == 3:
                # 转为单通道（取第一通道）
                m = m[:, :, 0]
            ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            out_path = save_dir / f'red_mask_{ts}.png'
            ok = cv2.imwrite(str(out_path), m)
            if not ok:
                print(f"[RedExtracter.save] imwrite 返回 False, 保存失败: {out_path}")
        except Exception as e:
            # 打印错误但不抛出，避免影响 ROS 线程
            print(f"[RedExtracter.save] 保存掩膜失败: {e}")

    def extract_red_points(self, img):
        """按顺序调用 yolo_mask 与 segment_mask，返回应用掩膜后的图像。

        参数:
            img: BGR 图像
        返回:
            masked_img: 应用最终掩膜后的 BGR 图像（numpy.ndarray）
        """
        if img is None:
            raise ValueError('extract_red_points: img is None')

        # 颜色分割掩膜
        mask = self.segment_mask(img)
        self.save(mask) # 调试

        # 应用掩膜并返回圖像
        masked_img = cv2.bitwise_and(img, img, mask=mask)
        return masked_img


    def red_cone_fit(self, mask, show_windows=True):
        """
        处理流程说明：
        1. 对输入的二值掩膜做连通域聚类（每个簇是一组空间相连的点）。
        2. 对每个簇，遍历所有点，使用 calc_X(v) 公式计算每个点的“深度”值（v为像素行号）。
        3. 计算该簇所有点深度的第5百分位（d0），只保留深度 < d0+margin 的点（margin默认0.2米）。
        4. 在可视化窗口标注第5百分位点（黄色圆点+数值），便于调试和理解分布。
        5. 最后再剔除图像上半部分的点（y < h//2），只保留下半部分。
        6. 返回最终裁剪后的掩膜，并弹窗显示聚类、分位点和裁剪结果。
        """

        if mask is None:
            raise ValueError("mask 不能为空")

        # 计算像素行对应的深度（与 cal_xyz.calc_X 等价）
        def calc_X(v):
            fy = 543.72991943
            c_y = 247.10844421
            X = 0.11 * fy / abs(c_y - v) # TODO: 检查 c_y - v > 0
            return X * 0.55

        # 规范化输入掩膜为单通道二值 (0/255)
        m = mask
        if m.dtype == np.bool_:
            m = (m.astype(np.uint8) * 255)
        elif m.dtype != np.uint8:
            m = m.astype(np.uint8)
        if m.ndim == 3 and m.shape[2] == 3:
            m = cv2.cvtColor(m, cv2.COLOR_BGR2GRAY)
        bin_mask = (m > 127).astype(np.uint8)

        # 连通域聚类：将所有白色点分成若干个空间相连的簇
        num_labels, labels = cv2.connectedComponents(bin_mask, connectivity=8)

        # 可视化每个簇（不同颜色显示，便于调试）
        h, w = bin_mask.shape
        vis = np.zeros((h, w, 3), dtype=np.uint8)
        rng = np.random.RandomState(12345)
        colors = [(0, 0, 0)] + [tuple(int(x) for x in rng.randint(50, 255, size=3)) for _ in range(num_labels)]
        for lab in range(1, num_labels):
            vis[labels == lab] = colors[lab]

        if show_windows:
            cv2.imshow('clusters', vis)
            cv2.waitKey(0)

        # 输出掩膜 & 用于标注第5百分位点的可视化图
        clipped_mask = np.zeros_like(bin_mask, dtype=np.uint8)
        d0_vis = vis.copy()
        d0_points_all = []  # (x,y,d0,lab)

        # 处理每个簇：
        #   - 遍历所有点，计算深度
        #   - 计算第5百分位深度d0
        #   - 只保留深度 < d0+margin 的点
        #   - 标注分位点到可视化图
        for lab in range(1, num_labels):
            ys, xs = np.where(labels == lab)
            if ys.size == 0:
                continue

            coords = list(zip(xs, ys))
            depth_pairs = []  # (x,y,z)

            # 收集深度（使用 calc_X(v)，v = 像素行 y）
            for (x, y) in coords:
                try:
                    z_val = float(calc_X(y))
                except Exception:
                    # 单点计算失败则跳过该点
                    continue
                if np.isfinite(z_val) and z_val > 0:
                    depth_pairs.append((x, y, z_val))

            if len(depth_pairs) == 0:
                continue

            # 一次性筛选：只保留深度 < d0+margin 的点，无需迭代
            margin = 0.2
            depths_arr = np.array([p[2] for p in depth_pairs])
            d0 = float(np.percentile(depths_arr, 5))
            threshold = d0 + margin
            # 找到最接近 d0 的点用于标注（黄色圆点+数值）
            diffs = np.abs(depths_arr - d0)
            idx_f = int(np.argmin(diffs))
            x_d0, y_d0, z_d0 = depth_pairs[idx_f]
            d0_points_all.append((int(x_d0), int(y_d0), d0, lab))
            cv2.circle(d0_vis, (int(x_d0), int(y_d0)), 4, (0, 255, 255), -1)
            cv2.putText(d0_vis, f"{d0:.1f}", (int(x_d0)+6, int(y_d0)-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, cv2.LINE_AA)

            # 标记所有满足条件的点到输出掩膜
            for (x, y, z_val) in depth_pairs:
                if np.isfinite(z_val) and z_val < threshold:
                    clipped_mask[y, x] = 255

        # 剔除图像上半边的点（只保留下半部分，y >= h//2）
        h_img = clipped_mask.shape[0]
        clipped_mask[:h_img//2, :] = 0

        # 可视化第5百分位点与裁剪结果（弹窗显示）
        if show_windows:
            cv2.imshow('d0_points', d0_vis)
            cv2.waitKey(0)
            clipped_vis = (clipped_mask > 0).astype(np.uint8) * 255
            clipped_vis = cv2.cvtColor(clipped_vis, cv2.COLOR_GRAY2BGR)
            cv2.imshow('clipped', clipped_vis)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return clipped_mask

if __name__ == '__main__':
    extracter = RedExtracter()

    SHOW = True
    mask_path = '/Users/xuzhenyu/Desktop/reutrn_stage/Pav-S_ws/src/costmap_process/new_on_robot/red_mask_20251102_231556_329303.png'
    mask = cv2.imread(mask_path, cv2.IMREAD_UNCHANGED)
    # 先过滤掉点数小于100的簇
    mask_filtered = extracter.filter_small_clusters(mask, min_size=100)
    if SHOW:
        cv2.imshow('mask_filtered', mask_filtered)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    mask_clipped = extracter.red_cone_fit(mask_filtered)
    # if SHOW:
    #     cv2.imshow('clipped_mask', mask_clipped)
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()
    
