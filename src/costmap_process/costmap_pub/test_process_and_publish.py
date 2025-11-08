import cv2
import numpy as np

IMG_PATH = '/Pav-S_ws/src/costmap_process/costmap_pub/image.png'
OUT_PATH = '/Pav-S_ws/src/costmap_process/costmap_pub/red_points_mask.png'
TARGET_RGB = (220, 20, 60)

def main():
    img = cv2.imread(IMG_PATH)
    if img is None:
        print(f"图片读取失败: {IMG_PATH}")
        return
    h, w = img.shape[:2]
    # 只判定完全等于目标RGB的点
    mask = (img[:,:,2] == TARGET_RGB[0]) & (img[:,:,1] == TARGET_RGB[1]) & (img[:,:,0] == TARGET_RGB[2])
    vis_img = np.zeros((h, w, 3), dtype=np.uint8)
    vis_img[mask] = (255, 255, 255)  # 白色
    cv2.imwrite(OUT_PATH, vis_img)
    print(f'已保存结果图片到: {OUT_PATH}')

if __name__ == '__main__':
    main()