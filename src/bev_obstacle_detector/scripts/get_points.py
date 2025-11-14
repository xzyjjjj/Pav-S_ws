import cv2

# 1. 存储点击的点的列表
points = []

# 2. 鼠标回调函数
def click_event(event, u, v, flags, params):
    # 检查是否为左键点击
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked Point: u={u}, v={v}")
        points.append((u, v))
        
        # 在图像上画一个点来确认
        cv2.circle(img, (u, v), 5, (0, 0, 255), -1)
        cv2.imshow("Image Viewer", img)

# 3. 读取你的 PNG 文件
img = cv2.imread("/Pav-S_ws/src/bev_obstacle_detector/scripts/cmdvel.png")

if img is None:
    print("Error: Could not load image.")
else:
    # 4. 创建窗口并设置鼠标回调
    cv2.imshow("Image Viewer", img)
    cv2.setMouseCallback("Image Viewer", click_event)
    
    print("Click on the four corners of your calibration rectangle (Left-click, then press any key to exit).")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # 5. 打印最终结果
    print("\n--- Final Source Points (u, v) ---")
    print(points)
    
    # 提示：如果需要将其转换为 ROS 参数的 1D 列表：
    # flat_list = [coord for point in points for coord in point]
    # print(flat_list)