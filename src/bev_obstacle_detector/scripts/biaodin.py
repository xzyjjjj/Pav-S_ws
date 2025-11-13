import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

WINDOW_NAME = "Image Viewer (Click to get pixel coordinates)"

class ImageClickViewer(Node):
    def __init__(self):
        super().__init__("image_click_viewer")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10
        )
        self.latest_frame = None
        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
        self.get_logger().info("订阅 /camera/color/image_raw，按 q 退出。")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.latest_frame is not None:
            print(f"点击坐标: ({x}, {y})")

    def draw_center_line(self, img):
        h, w = img.shape[:2]
        x_mid = w // 2
        cv2.line(img, (x_mid, 0), (x_mid, h), (0, 255, 0), 1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageClickViewer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.latest_frame is not None:
                disp = node.latest_frame.copy()
                node.draw_center_line(disp)
                cv2.imshow(WINDOW_NAME, disp)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()