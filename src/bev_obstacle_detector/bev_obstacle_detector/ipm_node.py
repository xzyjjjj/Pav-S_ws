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
# 【新增】导入 QoS 配置文件以优化实时性
from rclpy.qos import QoSProfile, qos_profile_sensor_data

# 辅助类 IPMConfig 保持不变
class IPMConfig:
    """一个辅助类，用于存储每套IPM配置"""
    def __init__(self, node: Node, config_id: int):
        id_str = f"_{config_id}"
        
        self.bev_width = node.get_parameter(f'bev_width{id_str}').value
        self.bev_height = node.get_parameter(f'bev_height{id_str}').value
        self.bev_size = (self.bev_width, self.bev_height)

        src_pts_1d = np.float32(node.get_parameter(f'src_points{id_str}').value)
        dst_pts_1d = np.float32(node.get_parameter(f'dst_points{id_str}').value)

        self.src_pts = src_pts_1d.reshape(4, 2)
        dst_pts = dst_pts_1d.reshape(4, 2)
        
        # 将 H 矩阵也存储在类中
        self.H = cv2.getPerspectiveTransform(self.src_pts, dst_pts)
        
        world_width_m = node.get_parameter(f'world_width_m{id_str}').value
        world_height_m = node.get_parameter(f'world_height_m{id_str}').value
        
        self.meters_per_pixel_x = world_width_m / self.bev_width
        self.meters_per_pixel_y = world_height_m / self.bev_height
        
        self.origin_offset_x_m = node.get_parameter(f'origin_offset_x_m{id_str}').value
        self.origin_offset_y_m = node.get_parameter(f'origin_offset_y_m{id_str}').value

        node.get_logger().info(f"--- 成功加载 IPM 配置 {config_id} ---")
        node.get_logger().info(f"  BEV 尺寸 (w, h): ({self.bev_width}, {self.bev_height})")
        node.get_logger().info(f"  源点 (Src Pts): {self.src_pts.tolist()}")
        node.get_logger().info(f"  目标点 (Dst Pts): {dst_pts.tolist()}")
        node.get_logger().info(f"  物理尺寸 (w, h): ({world_width_m:.4f}m, {world_height_m:.4f}m)")
        node.get_logger().info(f"  原点偏移 (x, y): ({self.origin_offset_x_m:.4f}m, {self.origin_offset_y_m:.4f}m)")
        node.get_logger().info(f"  像素比例 (X Px/m): {self.meters_per_pixel_x:.6f}")
        node.get_logger().info(f"  像素比例 (Y Px/m): {self.meters_per_pixel_y:.6f}")
        node.get_logger().info(f"---------------------------------")

        
class IPMNode(Node):

    def __init__(self):
        super().__init__('ipm_node')
        
        # --- 1. 声明所有参数 ---
        # 通用参数
        base_parameters = [
            ('image_topic', '/camera/image_raw'),
            ('pointcloud_topic', '/bev/obstacles'),
            ('bev_debug_image_topic', '/bev/debug_image'),
            ('bev_to_cmd_vel_topic', '/bev/to_cmd_vel'),
            ('enable_vis', False),
            ('camera_matrix', Parameter.Type.DOUBLE_ARRAY),
            ('distortion_coeffs', Parameter.Type.DOUBLE_ARRAY),
            ('morph_kernel_size', 5)
        ]
        
        # 动态声明 IPM 参数
        ipm_param_names = [
            'bev_width', 'bev_height', 'src_points', 'dst_points',
            'world_width_m', 'world_height_m', 'origin_offset_x_m', 'origin_offset_y_m'
        ]
        ipm_param_types = { 
            'bev_width': 640, 
            'bev_height': 480, 
            'src_points': Parameter.Type.DOUBLE_ARRAY, 
            'dst_points': Parameter.Type.DOUBLE_ARRAY, 
            'world_width_m': 2.0, 
            'world_height_m': 1.0, 
            'origin_offset_x_m': 0.5, 
            'origin_offset_y_m': 0.6 
        }
        
        for i in [1, 2]:
            for name in ipm_param_names:
                param_type = ipm_param_types.get(name, float)
                base_parameters.append((f'{name}_{i}', param_type))

        # 【新增】声明静态 HSV 阈值参数
        hsv_param_names = [
            'hsv_lower_red1', 'hsv_upper_red1', 'hsv_lower_red2', 'hsv_upper_red2'
        ]
        for i in [1, 2]:
            for name in hsv_param_names:
                base_parameters.append((f'{name}_{i}', Parameter.Type.INTEGER_ARRAY))

        self.declare_parameters(namespace='', parameters=base_parameters)

        # --- 2. 获取通用参数 ---
        self.enable_vis = self.get_parameter('enable_vis').value
        self.image_topic = self.get_parameter('image_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.bev_debug_image_topic = self.get_parameter('bev_debug_image_topic').value
        self.bev_to_cmd_vel_topic = self.get_parameter('bev_to_cmd_vel_topic').value

        kernel_size = self.get_parameter('morph_kernel_size').value
        # 【修正】使用方形内核
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))

        self.camera_matrix = np.array(self.get_parameter('camera_matrix').value).reshape(3, 3)
        self.dist_coeffs = np.array(self.get_parameter('distortion_coeffs').value)
        
        # --- 3. 加载两套 IPM 配置 ---
        self.ipm_configs = []
        try:
            self.ipm_configs.append(IPMConfig(self, 1))
            self.ipm_configs.append(IPMConfig(self, 2))
        except Exception as e:
            self.get_logger().error(f"加载 IPM 配置失败: {e}. 请检查你的 YAML 文件。")
            return

        # --- 4. 【新增】初始化 GPU 资源 ---
        self.get_logger().info("正在将资源上传到 GPU...")
        try:
            # 创建 CUDA 流
            self.stream = cv2.cuda.Stream()
            
            # 上传相机参数
            self.gpu_camera_matrix = cv2.cuda_GpuMat(self.camera_matrix)
            self.gpu_dist_coeffs = cv2.cuda_GpuMat(self.dist_coeffs)
            
            # 上传 IPM 变换矩阵
            self.gpu_H_1 = cv2.cuda_GpuMat(self.ipm_configs[0].H)
            self.gpu_H_2 = cv2.cuda_GpuMat(self.ipm_configs[1].H)

            # 上传形态学内核
            self.gpu_morph_open = cv2.cuda.createMorphologyFilter(cv2.MORPH_OPEN, cv2.CV_8U, self.morph_kernel)
            self.gpu_morph_close = cv2.cuda.createMorphologyFilter(cv2.MORPH_CLOSE, cv2.CV_8U, self.morph_kernel)

            # 上传静态 HSV 阈值
            self.gpu_lower_red1_1 = cv2.cuda_GpuMat(np.array(self.get_parameter('hsv_lower_red1_1').value, dtype=np.uint8))
            self.gpu_upper_red1_1 = cv2.cuda_GpuMat(np.array(self.get_parameter('hsv_upper_red1_1').value, dtype=np.uint8))
            self.gpu_lower_red2_1 = cv2.cuda_GpuMat(np.array(self.get_parameter('hsv_lower_red2_1').value, dtype=np.uint8))
            self.gpu_upper_red2_1 = cv2.cuda_GpuMat(np.array(self.get_parameter('hsv_upper_red2_1').value, dtype=np.uint8))
            
            self.gpu_lower_red1_2 = cv2.cuda_GpuMat(np.array(self.get_parameter('hsv_lower_red1_2').value, dtype=np.uint8))
            self.gpu_upper_red1_2 = cv2.cuda_GpuMat(np.array(self.get_parameter('hsv_upper_red1_2').value, dtype=np.uint8))
            self.gpu_lower_red2_2 = cv2.cuda_GpuMat(np.array(self.get_parameter('hsv_lower_red2_2').value, dtype=np.uint8))
            self.gpu_upper_red2_2 = cv2.cuda_GpuMat(np.array(self.get_parameter('hsv_upper_red2_2').value, dtype=np.uint8))
            
            self.get_logger().info("GPU 资源初始化成功。")
            
        except cv2.error as e:
            self.get_logger().fatal(f"OpenCV CUDA 初始化失败: {e}")
            self.get_logger().fatal("请确保 OpenCV 是在启用 CUDA 的情况下编译的！")
            return

        # --- 5. 初始化 ROS 接口 (使用 QoS) ---
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, self.pointcloud_topic, qos_profile_sensor_data
        )
        self.bev_image_pub = self.create_publisher(
            Image, self.bev_debug_image_topic, qos_profile_sensor_data
        )
        self.bev_to_cmd_vel_pub = self.create_publisher(
            Image, self.bev_to_cmd_vel_topic, 10
        )
        
    def destroy_node(self):
        if self.enable_vis:
            cv2.destroyAllWindows()
        super().destroy_node()

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge 转换失败: {e}')
            return

        # --- 【GPU】1. 上传图像到 GPU ---
        gpu_frame = cv2.cuda_GpuMat(cv_image)

        # --- 【GPU】2. 畸变校正 ---
        gpu_undistorted = cv2.cuda.undistort(
            gpu_frame, self.gpu_camera_matrix, self.gpu_dist_coeffs, stream=self.stream
        )

        # --- 【GPU】3. 逆透视变换 (IPM) ---
        gpu_bev_1 = cv2.cuda.warpPerspective(
            gpu_undistorted, self.gpu_H_1, self.ipm_configs[0].bev_size, flags=cv2.INTER_LINEAR, stream=self.stream
        )
        gpu_bev_2 = cv2.cuda.warpPerspective(
            gpu_undistorted, self.gpu_H_2, self.ipm_configs[1].bev_size, flags=cv2.INTER_LINEAR, stream=self.stream
        )

        # --- 【GPU】4. 颜色分割 (提取红色) ---
        gpu_hsv_1 = cv2.cuda.cvtColor(gpu_bev_1, cv2.COLOR_BGR2HSV, stream=self.stream)
        gpu_hsv_2 = cv2.cuda.cvtColor(gpu_bev_2, cv2.COLOR_BGR2HSV, stream=self.stream)
        
        # --- BEV 1 Mask ---
        gpu_mask1_1 = cv2.cuda.inRange(gpu_hsv_1, self.gpu_lower_red1_1, self.gpu_upper_red1_1, stream=self.stream)
        gpu_mask2_1 = cv2.cuda.inRange(gpu_hsv_1, self.gpu_lower_red2_1, self.gpu_upper_red2_1, stream=self.stream)
        gpu_red_mask_1 = cv2.cuda.bitwise_or(gpu_mask1_1, gpu_mask2_1, stream=self.stream)

        # --- BEV 2 Mask ---
        gpu_mask1_2 = cv2.cuda.inRange(gpu_hsv_2, self.gpu_lower_red1_2, self.gpu_upper_red1_2, stream=self.stream)
        gpu_mask2_2 = cv2.cuda.inRange(gpu_hsv_2, self.gpu_lower_red2_2, self.gpu_upper_red2_2, stream=self.stream)
        gpu_red_mask_2 = cv2.cuda.bitwise_or(gpu_mask1_2, gpu_mask2_2, stream=self.stream)
        
        # --- 【GPU】5. 形态学操作 ---
        gpu_red_mask_1 = self.gpu_morph_open.apply(gpu_red_mask_1, stream=self.stream)
        gpu_red_mask_1 = self.gpu_morph_close.apply(gpu_red_mask_1, stream=self.stream)
        gpu_red_mask_2 = self.gpu_morph_open.apply(gpu_red_mask_2, stream=self.stream)
        gpu_red_mask_2 = self.gpu_morph_close.apply(gpu_red_mask_2, stream=self.stream)

        # --- 【CPU】6. 下载结果 ---
        # 等待所有 GPU 操作完成
        self.stream.waitForCompletion()
        
        # 将二值 mask 下载回 CPU
        red_mask_1 = gpu_red_mask_1.download()
        red_mask_2 = gpu_red_mask_2.download()

        # --- 【CPU】7. 查找障碍物像素 ---
        obstacle_pixels_1 = cv2.findNonZero(red_mask_1)
        obstacle_pixels_2 = cv2.findNonZero(red_mask_2)

        # --- 【CPU】8. 转换为 PointCloud2 ---
        points_list = []
        cfg1 = self.ipm_configs[0]
        if obstacle_pixels_1 is not None:
            for point in obstacle_pixels_1:
                u, v = point[0]
                robot_x = cfg1.origin_offset_x_m + (cfg1.bev_height - v) * cfg1.meters_per_pixel_y
                robot_y = cfg1.origin_offset_y_m - u * cfg1.meters_per_pixel_x
                points_list.append([robot_x, 0.0, robot_y]) # Z=0.0

        cfg2 = self.ipm_configs[1]
        if obstacle_pixels_2 is not None:
            for point in obstacle_pixels_2:
                u, v = point[0]
                robot_x = cfg2.origin_offset_x_m + (cfg2.bev_height - v) * cfg2.meters_per_pixel_y
                robot_y = cfg2.origin_offset_y_m - u * cfg2.meters_per_pixel_x
                points_list.append([robot_x, 0.0, robot_y])

        # --- 【CPU】9. 创建并发布点云 ---
        header = Header(stamp=msg.header.stamp, frame_id="body") # 使用输入图像的时间戳
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        point_cloud_msg = point_cloud2.create_cloud(header, fields, points_list)
        # self.pointcloud_pub.publish(point_cloud_msg)
            
        # --- 【CPU】10. 可选的可视化和调试图像发布 ---
        if self.enable_vis or self.bev_image_pub.get_subscription_count() > 0:
            # 只有在需要时才下载 BEV 图像
            bev_image_1 = gpu_bev_1.download()
            bev_image_2 = gpu_bev_2.download()
            
            # 发布调试图像 (Config 1)
            debug_bev_display = cv2.bitwise_and(bev_image_1, bev_image_1, mask=red_mask_1)
            bev_img_msg = self.bridge.cv2_to_imgmsg(debug_bev_display, "bgr8")
            bev_img_msg.header = header
            self.bev_image_pub.publish(bev_img_msg)

            self.bev_to_cmd_vel_pub.publish(bev_img_msg)
            if self.enable_vis:
                # 显示原始图像 (需要下载)
                undistorted_img = gpu_undistorted.download()
                vis_img = undistorted_img.copy()
                for pt in cfg1.src_pts:
                     cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 5, (0, 255, 0), -1)
                for pt in cfg2.src_pts:
                     cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
                cv2.imshow("1. Undistorted Image (CPU)", vis_img)
                
                # 显示 BEV 结果
                debug_bev_display_2 = cv2.bitwise_and(bev_image_2, bev_image_2, mask=red_mask_2)
                cv2.imshow("2.1 BEV Result (Config 1)", debug_bev_display)
                cv2.imshow("2.2 BEV Result (Config 2)", debug_bev_display_2)
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    ipm_node = IPMNode()
    rclpy.spin(ipm_node)
    ipm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()