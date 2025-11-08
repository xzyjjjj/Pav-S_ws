"""
cal_xyz.py

X 朝前，Y 朝左，Z 朝上。

功能：
- 提供从图像像素坐标 (u, v) 计算相机坐标系下地面点 (X, Z) 的方法。
- 主要用于将检测框或像素点投影到相机坐标系，便于后续 map 坐标变换。
- 依赖相机内参（CAMERA, CAMERA_PRIORI），支持鼠标交互测试。

核心函数：
- calc_Z(v): 由像素 v 计算深度 Z。
- calc_X(u, v, Z): 由像素 u, v 和深度 X 计算横向 Y。
"""
import cv2
from .config import CAMERA, CAMERA_PRIORI

class calculate_xyz:
	def __init__(self):
		pass

	def calc_X(self, v):
		# v_0 = CAMERA_PRIORI['v_0']
		# X_0 = CAMERA_PRIORI['X_0']
		fy = CAMERA['fy']
		c_y = CAMERA['cy']
		# X = X_0 * abs(c_y - v_0) / abs(c_y - v)
		X = CAMERA_PRIORI['height'] * fy / abs(c_y - v) # TODO: 检查 c_y - v > 0
		return X * 0.55

	def calc_Y(self, u, v, X):
		fx = CAMERA['fx']
		cx = CAMERA['cx']
		cy = CAMERA['cy']
		numerator = (CAMERA_PRIORI['height']**2 + X**2) ** 0.5
		denominator = (fx**2 + (cy - v)**2) ** 0.5
		Y = numerator / denominator * abs(cx - u)
		return -Y
	def calc_xyz(self, u, v):
		# 使用实例方法调用，保证 self 正确传递给 calc_X 和 calc_Y
		X = self.calc_X(v)
		Y = self.calc_Y(u, v, X)
		return X, Y, -CAMERA_PRIORI['height']

# class cv2_tools:
# 	def __init__(self, img_path):
# 		self.img_path = img_path

	# def show_image(self):
	# 	img = cv2.imread(self.img_path)
	# 	if img is None:
	# 		print(f"图片未找到: {self.img_path}")
	# 		return None, None, None
	# 	h, w = img.shape[:2]
	# 	# 在图片 img 上画一条垂直的红色直线，位置在图像的水平中心（w//2），从顶部 (w//2, 0) 到底部 (w//2, h)，颜色为红色 (0, 0, 255)，线宽为 2 像素。
	# 	cv2.line(img, (w//2, 0), (w//2, h), (0, 0, 255), 2)
	# 	return img, w, h

	# def mouse_callback(event, x, y, flags, param):
	# 	if event == cv2.EVENT_LBUTTONDOWN:
	# 		X, Z = solve_xyz(x, y)
	# 		print(f"点击位置u={x}, v={y}, X={X:.6f}, Z={Z:.6f}")
			
# def main():
# 	img_path = 'cal_Z_assets/1.png'
# 	img, w, h = show_image(img_path)
# 	if img is None:
# 		return
# 	print("请点击图片，获取像素(u, v)，并返回X, Z，按ESC退出")
# 	cv2.namedWindow('IMG')
# 	cv2.setMouseCallback('IMG', mouse_callback)
# 	while True:
# 		cv2.imshow('IMG', img)
# 		if cv2.waitKey(10) == 27:
# 			break
# 	cv2.destroyAllWindows()

if __name__ == '__main__':
	# main()
	pass