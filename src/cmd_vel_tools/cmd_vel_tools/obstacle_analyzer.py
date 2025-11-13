import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from red_segment_msg.msg import ObstacleInfo
from geometry_msgs.msg import Twist

class ObstacleAnalyzerNode(Node):
    def __init__(self):
        super().__init__('obstacle_analyzer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/bev/to_cmd_vel',
            self.image_callback,
            10)
        
        self.subscription_cmd = self.create_subscription(
            Twist,
            '/cmd_vel_nav2',  
            self.cmd_vel_callback,
            10)
        self.current_angular_z = 0.1  # 存储当前角速度

        self.publisher_ = self.create_publisher(
            ObstacleInfo,
            '/bev/obstacle_info', 
            10)
        # PID 参数
        self.declare_parameter('kp_angular', 0.0008) # 比例增益 (P)
        self.declare_parameter('kd_angular', 0.1)    # 微分增益 (D) (阻尼)

        # 远处的权重 (避免过度反应)
        self.declare_parameter('dist_weight_far', 0.5) 
        # 近处的权重 (需要紧急避让)
        self.declare_parameter('dist_weight_near', 1.5) 
        self.w_far = self.get_parameter('dist_weight_far').value
        self.w_near = self.get_parameter('dist_weight_near').value

        # 障碍检测参数
        self.declare_parameter('min_obstacle_area', 20)
        self.declare_parameter('min_lane_area', 100)
        self.declare_parameter('min_total_lane_area', 500)


        self.kp_angular = self.get_parameter('kp_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        self.min_obs_area = self.get_parameter('min_obstacle_area').value
        self.min_lane_area = self.get_parameter('min_lane_area').value
        self.min_total_lane_area = self.get_parameter('min_total_lane_area').value
        self.min_area_threshold = self.get_parameter('min_obstacle_area').value
        self.get_logger().info(f'Obstacle Analyzer Node inited. Min area: {self.min_area_threshold}px')

    def cmd_vel_callback(self, msg: Twist):
        self.current_angular_z = msg.angular.z

    def image_callback(self, msg: Image):
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge 转换失败: {e}')
            return

        # "步骤 1: 正在提取红色障碍物..."
        red_mask = create_red_mask(cv_image, morph_kernel_size=5)

        # "步骤 2: 正在提取黑色赛道 (使用形状过滤)..."
        black_mask = create_black_mask(cv_image, morph_kernel_size=5)

        # "步骤 3: 正在分析相对位置..."
        results, cleaned_mask = analyze_obstacle_image(
            red_mask,
            black_mask,
            self.min_obs_area,
            self.min_lane_area,
            self.min_total_lane_area
        )
        if results is None:
            return

        msg_out = ObstacleInfo()
        msg_out.left = msg_out.right = False
        msg_out.find = results['obstacle_found']
        if results['obstacle_found']:
            if results['decision'] == "TURN_LEFT":
                msg_out.left = True
            elif results['decision'] == "TURN_RIGHT":
                msg_out.right = True
                
        # ------ 4. (核心) PD 控制器逻辑 (*** 最终版 ***) -------
        target_angular_z = 0.0
        pixel_error = 0.0
        critical_y = 0

        if msg_out.left:
            lane_reference_x = cv_image.shape[1] - 1
        else : 
            lane_reference_x = 0.0
            
        
        # 2. (核心) 根据 "analyze" 函数的决策，计算误差
        # PD 控制器 *执行* 决策，而不是 *推翻* 决策
        if results['decision'] == "TURN_LEFT":
            # 决策: 向左转 (意味着障碍物在右侧, 或在赛道左侧)
            pixel_error = lane_reference_x - results['left_x']       

        elif results['decision'] == "TURN_RIGHT":
            # 决策: 向右转 (意味着障碍物在左侧, 或在赛道右侧)
            pixel_error = lane_reference_x - results['right_x']
            
        # 像素底部距离加权
        if msg_out.find:
            obs_bottom_y = results.get('bottom_y', 0.0)
            img_height = results.get('img_height', 100)
            normalized_dist = obs_bottom_y / float(img_height)    # [0,1]
            distance_weight = self.w_far + (self.w_near - self.w_far) * normalized_dist

        # P 项 (比例):
        target_angular_z = self.kp_angular * pixel_error * distance_weight

        # D 项 (阻尼): 
        dampening_term = self.kd_angular * self.current_angular_z
        
        # 最终命令 = P项 - D项
        final_angular_command = target_angular_z - dampening_term
        
        # 填充消息
        if msg_out.find:
            msg_out.angular_change = final_angular_command
            if results['lane_found']:
                self.get_logger().info(f'Lane found at y={critical_y}') 
            self.get_logger().info(f'Computed angular_change: {msg_out.angular_change:.3f} rad/s')
            self.get_logger().info(f'pixel_error: {pixel_error:.1f} px')
        else:
            msg_out.angular_change = 0.0

        # 8. 发布消息
        self.publisher_.publish(msg_out)
        
        
        # --- (更新) 打印分析结果 ---
        # print(f"  障碍物检测: {results['obstacle_found']}")
        # print(f"  赛道线检测: {results['lane_found']}")

        # if results['lane_found']:
        #     print(f"  (新逻辑) 障碍物 vs 赛道曲线:")
        #     print(f"  - 面积 (左侧): {results.get('area_left_of_lane', 0)} px")
        #     print(f"  - 面积 (右侧): {results.get('area_right_of_lane', 0)} px")
        #     print(f"  - 主导侧: {results.get('dominant_obstacle_side_vs_lane', 'N/A')}")
        # elif results['obstacle_found']:
        #     print(f"  (回退逻辑) 障碍物 vs 图像中线:")
        #     print(f"  - 边界 X: [{results.get('left_x', 0.0)}, {results.get('right_x', 0.0)}]")
        # else:
        #     print("  (无障碍物)")

        # print("---")
        # print(f"  决策理由: {results.get('decision_reason', 'N/A')}")
        # print(f"  最终决策: {results.get('decision', 'GO_STRAIGHT')}")
        # print("----------------------------")

        # # --- 创建可视化 ---
        # vis_image = bev_image.copy()
        # h = results['img_height']
        # w = results['img_width']
        # mid_x = results['mid_x_pixel']

        # # 绘制中线 (仅供参考)
        # cv2.line(vis_image, (mid_x, 0), (mid_x, h), (0, 255, 0), 1)

        # # 绘制 *干净的* 掩码 (包含障碍物和 *所有* 赛道点)
        # # 黑色赛道点现在是蓝色的
        # # 红色障碍物点现在是红色的
        # vis_image[cleaned_mask > 0] = (255, 0, 0)  # 蓝色 (赛道)
        # vis_image[red_mask > 0] = (0, 0, 255)  # 红色 (障碍物)

        # if results['obstacle_found']:
        #     lx_obs = int(results['left_x'])
        #     rx_obs = int(results['right_x'])
        #     # 用绿色框出障碍物
        #     cv2.rectangle(vis_image, (lx_obs, 0), (rx_obs, h), (0, 255, 0), 2)

        # # (移除 'fitLine' 绘制逻辑)

        # cv2.putText(vis_image, f"DECISION: {results['decision']}", (mid_x - 100, h - 15),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # # --- (修改) 显示所有中间图像 ---
        # cv2.imshow("1. Input (Original Camera Image)", original_camera_image)
        # cv2.imshow("2. Undistorted Image", undistorted_image)
        # cv2.imshow("3. Generated BEV Image", bev_image)
        # cv2.imshow("4. Generated Red Mask (Obstacles)", red_mask)
        # cv2.imshow("5. Generated Black Mask (Lanes) (Filtered)", black_mask)  # (标题更新)
        # cv2.imshow("6. Cleaned Combined Mask (Obs+AllLanes)", cleaned_mask)
        # cv2.imshow("7. Final Analysis (on BEV)", vis_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()


# 函数:红色掩码生成
def create_red_mask(bev_image, morph_kernel_size=5):
    """
    模拟 ipm_node.py 中的第 3 步和第 4 步 (红色)。
    """
    hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
    mean_v = np.mean(hsv[:, :, 2])
    v_thresh = max(50, int(mean_v * 0.5))

    lower_red_1 = np.array([0, 90, v_thresh])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([170, 80, v_thresh])
    upper_red_2 = np.array([180, 255, 255])

    hsv_image = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    morph_kernel = np.ones((1, morph_kernel_size), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, morph_kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, morph_kernel)

    return red_mask

def find_dominant_track_side(black_mask, min_pixel_thresh=80):
    """
    分析黑色赛道掩码，判断赛道主要在左侧、右侧还是中间。
    """

    # 获取图像宽度并计算中点
    h, w = black_mask.shape
    mid_x = w // 2

    # 分割掩码为左右两半
    left_mask = black_mask[:, 0:mid_x]
    right_mask = black_mask[:, mid_x:w]

    # 统计两侧的 "赛道" 像素数量
    left_pixels = cv2.countNonZero(left_mask)
    right_pixels = cv2.countNonZero(right_mask)

    total_pixels = left_pixels + right_pixels

    
    # 如果总像素太少 (可能是噪声)，返回 "none"
    if total_pixels < min_pixel_thresh:
        return "none" 
    left_ratio = left_pixels / total_pixels
    if left_ratio > 0.5:
        return "left"
    else:
        return "right"


# 函数: 黑色赛道掩码生成 (鲁棒)
def create_black_mask(bev_image, morph_kernel_size=5):  # morph_kernel_size 实际上被覆盖了
    """
    1. 使用 HSV 阈值检测 *黑色* 赛道
    2. 增加轮廓形状分析，过滤掉 "圆形" 噪声 (如数字标签)
    """
    # 1. 转换为 HSV
    hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)

    # 2. (核心) 定义 "黑色" 的 HSV 范围
    mean_v = np.mean(hsv[:, :, 2])
    v_thresh = max(70, int(mean_v * 0.5))
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 30, v_thresh])

    black_mask_raw = cv2.inRange(hsv, lower_black, upper_black)

    # 3. (新) 轮廓过滤
    # 在执行大型闭合操作之前，先过滤掉非曲线形状

    # 3a. 先用一个小的 "开" 操作移除噪声
    open_kernel_size = 3
    kernel_open_small = cv2.getStructuringElement(cv2.MORPH_RECT, (open_kernel_size, open_kernel_size))
    black_mask_opened = cv2.morphologyEx(black_mask_raw, cv2.MORPH_OPEN, kernel_open_small)

    # 3b. 查找所有剩余轮廓
    contours, _ = cv2.findContours(black_mask_opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_mask = np.zeros_like(black_mask_raw)

    # 3c. 定义 "曲线" (赛道) 的标准
    min_contour_area = 50  # 过滤掉太小的噪声
    min_elongation_ratio = 2.5  # 必须是 2.5:1 (长:宽 或 宽:长)

    good_contours = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_contour_area:
            continue
        # (核心) 使用 *旋转* 边界框 (minAreaRect) 来准确获取"伸长率"
        # 这比 boundingRect (x,y,w,h) 更适合倾斜的赛道线
        try:
            # minAreaRect 返回 ((center_x, center_y), (width, height), angle)
            _, (w, h), _ = cv2.minAreaRect(c)
        except:
            # 如果轮廓点太少 (例如 1-2 个像素)，minAreaRect 可能会失败
            continue

        if w == 0 or h == 0:
            continue

        # 计算长宽比 (确保取 "长边 / 短边")
        aspect_ratio = max(h / w, w / h)

        # 如果它足够 "长" (长 > 2.5*宽)，就保留它
        if aspect_ratio > min_elongation_ratio:
            good_contours.append(c)
        # else:
        # (调试时取消注释) 打印被过滤掉的轮廓信息
        # print(f"info: Filtering out round-ish contour. Area: {area}, W: {w:.1f}, H: {h:.1f}, Ratio: {aspect_ratio:.2f}")

    if not good_contours:
        return filtered_mask  # 如果没有好的轮廓，返回空掩码

    # 3d. 重新绘制 *只* 包含 "好" 轮廓的掩码
    cv2.drawContours(filtered_mask, good_contours, -1, 255, thickness=cv2.FILLED)

    # 4. 形态学操作 (在 *过滤后* 的掩码上)
    # (A) (保留) 使用 "闭运算" (CLOSE) 来连接断开的 *赛道线段*
    close_kernel_size = 50
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (close_kernel_size, close_kernel_size))
    black_mask_closed = cv2.morphologyEx(filtered_mask, cv2.MORPH_CLOSE, kernel_close)

    # (B) (保留) 在 "闭运算" 之后，使用一个较小的 "开运算" (OPEN) 来移除连接过程中产生的小噪点
    # (复用 3x3 kernel)
    final_black_mask = cv2.morphologyEx(black_mask_closed, cv2.MORPH_OPEN, kernel_open_small)

    return final_black_mask


def analyze_obstacle_image(obstacle_mask, lane_mask, min_obstacle_area=100, min_lane_area=5, min_total_lane_area=100):

    h, w = obstacle_mask.shape[:2]
    results = {}
    mid_x_pixel = w // 2  # <-- BUG 1 修复: 在顶部定义

    # --- A. 分析障碍物 (Obstacles) ---
    contours_obs, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    significant_obs_contours = []
    for contour in contours_obs:
        if cv2.contourArea(contour) > min_obstacle_area:
            significant_obs_contours.append(contour)

    if not significant_obs_contours:
        # 1. (回退) 没有障碍物
        results['obstacle_found'] = False
        results['lane_found'] = False # (假设无障碍也无需检查赛道)
        results['decision'] = "GO_STRAIGHT"
        results['decision_reason'] = "No Obstacle Found"
        results['left_x'] = 0.0
        results['right_x'] = 0.0
        results['bottom_y'] = 0.0
        results['mid_x_pixel'] = mid_x_pixel
        results['img_height'] = h
        results['img_width'] = w
        
        # 修正逻辑: 如果没有障碍物, 返回的 "cleaned_mask" 应该是赛道掩码
        return results, lane_mask

    # 找到障碍物了
    results['obstacle_found'] = True
    cleaned_obs_mask = np.zeros_like(obstacle_mask)
    cv2.drawContours(cleaned_obs_mask, significant_obs_contours, -1, 255, thickness=cv2.FILLED)

    all_obs_points = np.concatenate(significant_obs_contours)
    x_obs_coords = all_obs_points[:, 0, 0]
    y_obs_coords = all_obs_points[:, 0, 1]
    results['left_x'] = float(np.min(x_obs_coords))
    results['right_x'] = float(np.max(x_obs_coords))
    results['bottom_y'] = float(np.max(y_obs_coords))

    # --- B. 分析赛道 (Lanes) ---
    
    # *** BUG 2 修复: 在这里定义 combined_cleaned_mask ***
    # 这是你想要的掩码: 合并了"有效障碍物" (cleaned_obs_mask) 和 "有效赛道" (lane_mask)
    combined_cleaned_mask = cv2.bitwise_or(cleaned_obs_mask, lane_mask)

    # (现在 mid_x_pixel 已经定义了)
    is_find_lane = find_dominant_track_side(lane_mask, min_pixel_thresh=50)
    left_obs_mask = cleaned_obs_mask[:, :mid_x_pixel]
    right_obs_mask = cleaned_obs_mask[:, mid_x_pixel:]
    left_obs_area = cv2.countNonZero(left_obs_mask)
    right_obs_area = cv2.countNonZero(right_obs_mask)

    if is_find_lane == "none":
        # 2. (回退) 有障碍物，但没有赛道 (或赛道点太少无法拟合)
        results['lane_found'] = False
        results['decision_reason'] = "Fallback: No Lane Found (or Lane too small)"
        if left_obs_area > right_obs_area:
            results['decision'] = "TURN_RIGHT"
        else:
            results['decision'] = "TURN_LEFT"

        results['mid_x_pixel'] = mid_x_pixel
        results['img_height'] = h
        results['img_width'] = w
        
        # 修正逻辑: 即使没有赛道, 也要返回包含障碍物的 combined_mask
        return results, combined_cleaned_mask
    else:
        results['lane_found'] = True
        if is_find_lane == "left":
            # 由于视野受限，赛道在左侧时如果发现障碍，则一定右转
            results['decision'] = "TURN_RIGHT"
            results['decision_reason'] = "Context: Lane is DOMINANT on LEFT"
        else:
            results['decision'] = "TURN_LEFT"
            results['decision_reason'] = "Context: Lane is DOMINANT on RIGHT"
            
        # 修正逻辑: 决策完成后, 返回正确的 combined_mask
        return results, combined_cleaned_mask

def main(args=None):
    rclpy.init(args=args)
    analyzer_node = ObstacleAnalyzerNode()
    rclpy.spin(analyzer_node)
    analyzer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()