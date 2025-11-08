# bev_obstacle_detector/ipm_node.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs_py import point_cloud2

class IPMNode(Node):

    def __init__(self):
        super().__init__('ipm_node')

        # --- 1. 声明所有参数 (使用 1D 浮点数列表作为默认值) ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', '/camera/image_raw'),
                ('pointcloud_topic', '/bev/obstacles'),
                ('bev_debug_image_topic', '/bev/debug_image'),
                
                # 可视化控制参数
                ('enable_vis', False),  # 默认设置为 False

                # 相机参数 (1D 列表)
                ('camera_matrix', Parameter.Type.DOUBLE_ARRAY),
                ('distortion_coeffs', Parameter.Type.DOUBLE_ARRAY),
                
                # BEV 尺寸
                ('bev_width', 640),
                ('bev_height', 480),
                
                # IPM 点 (1D 浮点数列表)
                ('src_points', Parameter.Type.DOUBLE_ARRAY),
                ('dst_points', Parameter.Type.DOUBLE_ARRAY),

                # 物理尺寸
                ('world_width_m', 2.0),
                ('world_height_m', 3.0),
                ('origin_offset_x_m', 0.5),
                ('origin_offset_y_m', 1.0),

                # # HSV (1D 整数列表)
                # ('hsv_lower_red1', Parameter.Type.INTEGER_ARRAY),
                # ('hsv_upper_red1', Parameter.Type.INTEGER_ARRAY),
                # ('hsv_lower_red2', Parameter.Type.INTEGER_ARRAY),
                # ('hsv_upper_red2', Parameter.Type.INTEGER_ARRAY),
                
                ('morph_kernel_size', 5)
            ]
        )

        # --- 2. 获取所有参数 ---
        self.enable_vis = self.get_parameter('enable_vis').value

        # 话题
        self.image_topic = self.get_parameter('image_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.bev_debug_image_topic = self.get_parameter('bev_debug_image_topic').value

        # BEV 尺寸
        self.bev_width = self.get_parameter('bev_width').value
        self.bev_height = self.get_parameter('bev_height').value
        self.bev_size = (self.bev_width, self.bev_height)

        # 物理尺寸
        world_width_m = self.get_parameter('world_width_m').value
        world_height_m = self.get_parameter('world_height_m').value
        
        # 坐标系
        self.origin_offset_x_m = self.get_parameter('origin_offset_x_m').value
        self.origin_offset_y_m = self.get_parameter('origin_offset_y_m').value
        
        # # HSV (转换为 NumPy 数组)
        # self.lower_red1 = np.array(self.get_parameter('hsv_lower_red1').value)
        # self.upper_red1 = np.array(self.get_parameter('hsv_upper_red1').value)
        # self.lower_red2 = np.array(self.get_parameter('hsv_lower_red2').value)
        # self.upper_red2 = np.array(self.get_parameter('hsv_upper_red2').value)

        # 形态学
        kernel_size = self.get_parameter('morph_kernel_size').value
        self.morph_kernel = np.ones((1, kernel_size), np.uint8)

        # --- 3. 处理相机和 IPM 矩阵 ---
        self.camera_matrix = np.array(self.get_parameter('camera_matrix').value).reshape(3, 3)
        self.dist_coeffs = np.array(self.get_parameter('distortion_coeffs').value)
        
        src_pts_1d = np.float32(self.get_parameter('src_points').value)
        dst_pts_1d = np.float32(self.get_parameter('dst_points').value)

        # ** 关键：将 1D 列表重塑为 4x2 矩阵 **
        self.src_pts = src_pts_1d.reshape(4, 2)
        self.dst_pts = dst_pts_1d.reshape(4, 2)

        self.H = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)
        
        # --- 4. 计算物理比例 ---
        self.meters_per_pixel_x = world_width_m / self.bev_width
        self.meters_per_pixel_y = world_height_m / self.bev_height
        
        self.get_logger().info(f"IPM 节点启动。BEV 尺寸: {self.bev_width}x{self.bev_height}")
        self.get_logger().info(f"物理比例 Y (前进): {self.meters_per_pixel_y:.4f} m/pixel")
        self.get_logger().info(f"物理比例 X (侧向): {self.meters_per_pixel_x:.4f} m/pixel")

        # --- 5. 初始化 ROS 接口 ---
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, self.pointcloud_topic, 10
        )
        self.bev_image_pub = self.create_publisher(
            Image, self.bev_debug_image_topic, 10
        )

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge 转换失败: {e}')
            return

        # 1. 畸变校正
        # 注意: 如果您的相机驱动已经发布了 /camera/image_rect_color,
        # 您应该订阅那个话题并跳过此步以节省计算。
        undistorted_img = cv2.undistort(
            cv_image, self.camera_matrix, self.dist_coeffs, None, None
        )

        # 【新增】如果启用可视化，显示原始图像
        if self.enable_vis:
            # 复制图像并在上面画出 IPM 源点，方便调试
            vis_img = undistorted_img.copy()
            for pt in self.src_pts:
                 cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 5, (0, 255, 0), -1)
            cv2.imshow("1. Undistorted Image (with Src Pts)", vis_img)


        # 2. 逆透视变换 (IPM) -> 鸟瞰图
        bev_image = cv2.warpPerspective(
            undistorted_img, self.H, self.bev_size, flags=cv2.INTER_LINEAR
        )

        # 3. 颜色分割 (提取红色)
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
        
        # 4. 形态学操作 (去噪)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, self.morph_kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self.morph_kernel)

        # 5. 查找障碍物像素
        obstacle_pixels = cv2.findNonZero(red_mask)

        # 6. 转换为 PointCloud2
        points_list = []
        if obstacle_pixels is not None:
            for point in obstacle_pixels:
                u, v = point[0] # BEV 图像中的像素坐标 (u, v)
                
                # 将 (u, v) 转换为 base_link 中的坐标 (x, y)
                # base_link: x (前), y (左)
                # BEV 图像: u (右), v (下)
                
                # Y (前): v=bev_height 对应 x=origin_offset_x_m
                #           v=0 对应 x=origin_offset_x_m + world_height_m
                robot_x = self.origin_offset_x_m + (self.bev_height - v) * self.meters_per_pixel_y
                
                # X (左): u=0 对应 y=origin_offset_y_m
                #           u=bev_width 对应 y=origin_offset_y_m - world_width_m
                robot_y = self.origin_offset_y_m - u * self.meters_per_pixel_x
                
                robot_z = 0.0 # 假设所有障碍物在地面
                points_list.append([robot_x, robot_y, robot_z])

        # 7. 创建并发布点云
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        point_cloud_msg = point_cloud2.create_cloud(header, fields, points_list)
        self.pointcloud_pub.publish(point_cloud_msg)
            
        # 8. (可选) 发布调试图像
        debug_bev_display = cv2.bitwise_and(bev_image, bev_image, mask=red_mask)
        bev_img_msg = self.bridge.cv2_to_imgmsg(debug_bev_display, "bgr8")
        bev_img_msg.header = header # 使用相同的时间戳和 frame_id
        self.bev_image_pub.publish(bev_img_msg)

        # 【新增】如果启用可视化，显示鸟瞰图和处理结果
        if self.enable_vis:
            cv2.imshow("2. BEV Result (Whole Image)", bev_image)
            cv2.imshow("3. BEV Result (Color Overlay)", debug_bev_display)
            # 必须调用 waitKey 来刷新窗口和处理事件
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    ipm_node = IPMNode()
    rclpy.spin(ipm_node)
    ipm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()