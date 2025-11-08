import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from red_segment_msg.msg import ObstacleInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


def red_segment(img):
    """检测红色区域"""
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.equalizeHist(l)
    lab_eq = cv2.merge((l, a, b))
    img_eq = cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)

    hsv = cv2.cvtColor(img_eq, cv2.COLOR_BGR2HSV)
    mean_v = np.mean(hsv[:, :, 2])
    v_thresh = max(50, int(mean_v * 0.5))

    lower_red_1 = np.array([0, 90, v_thresh])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([170, 80, v_thresh])
    upper_red_2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    return cv2.bitwise_or(mask1, mask2)


def extract_obstacles(mask):
    """从掩膜中提取障碍物"""
    kernel = np.ones((18, 18), np.uint8)
    mask_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    mask_clean = mask_open.copy()
    for _ in range(5):
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask_clean, connectivity=8)
    obstacles = []
    for i in range(1, num_labels):
        x, y, w, h, area = stats[i]
        if area < 900:
            continue
        obstacles.append({
            "bbox": (x, y, w, h),
            "centroid": (int(centroids[i][0]), int(centroids[i][1])),
        })
    return obstacles, mask_clean


def detect_track_edges(frame, roi_y_ratio=0.5):
    """检测赛道左右边缘（黑色线）"""
    h, w = frame.shape[:2]
    y0 = int(h * roi_y_ratio)
    roi = frame[y0:, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray_eq = cv2.equalizeHist(gray)
    th = cv2.adaptiveThreshold(gray_eq, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY_INV, 15, 7)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel, iterations=2)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pts = np.vstack([c.reshape(-1, 2) for c in contours if cv2.contourArea(c) > 50]) if contours else None
    if pts is None or len(pts) < 50:
        return None, None, None

    pts[:, 1] += y0  # 转换到全局坐标
    median_x = np.median(pts[:, 0])
    left_pts = pts[pts[:, 0] < median_x]
    right_pts = pts[pts[:, 0] >= median_x]

    def fit_poly(points):
        if len(points) < 30:
            return None
        ys, xs = points[:, 1], points[:, 0]
        try:
            return np.poly1d(np.polyfit(ys, xs, 2))
        except:
            return None

    fit_left = fit_poly(left_pts)
    fit_right = fit_poly(right_pts)

    def evaluate_x(y):
        x_left = fit_left(y) if fit_left is not None else None
        x_right = fit_right(y) if fit_right is not None else None
        center = None
        if x_left is not None and x_right is not None:
            center = (x_left + x_right) / 2.0
        elif x_left is not None:
            center = x_left + w * 0.25
        elif x_right is not None:
            center = x_right - w * 0.25
        return x_left, x_right, center

    return evaluate_x, fit_left, fit_right


class RedSegmentNode(Node):
    def __init__(self):
        super().__init__('red_segment')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('obstacle_topic', '/obstacle_cv')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        obstacle_topic = self.get_parameter('obstacle_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.obstacle_pub = self.create_publisher(ObstacleInfo, obstacle_topic, 10)
        self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.get_logger().info("RedSegmentNode initialized")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV bridge fault: {e}")
            return

        mask = red_segment(frame)
        obstacles, _ = extract_obstacles(mask)
        h_img, w_img = frame.shape[:2]

        eval_track, _, _ = detect_track_edges(frame, roi_y_ratio=0.5)
        trigger, left, right = False, False, False

        for obs in obstacles:
            x, y, w, h = obs["bbox"]
            mid_x = x + w / 2
            bottom_y = y + h
            if bottom_y < h_img * 0.9:
                continue  # 太远，忽略
            trigger = True
            if eval_track is not None:
                xl, xr, center = eval_track(bottom_y)
                if xl and xr and center:
                    if mid_x < center:
                        left = True
                    else:
                        right = True
                else:
                    # 如果赛道线检测失败，退化为图像左右判断
                    left = mid_x < w_img / 2
                    right = not left
            else:
                left = mid_x < w_img / 2
                right = not left
            break

        msg_out = ObstacleInfo()
        msg_out.bottom = trigger
        msg_out.left = left
        msg_out.right = right
        if trigger:
            side = "LEFT" if left else "RIGHT"
            self.get_logger().info(f"Obstacle detected on {side}")
        self.obstacle_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = RedSegmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
