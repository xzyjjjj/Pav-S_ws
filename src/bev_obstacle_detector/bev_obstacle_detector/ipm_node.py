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
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
# --- 【PyTorch】新增导入 ---
try:
    import torch
    import torch.nn.functional as F
except ImportError:
    print("="*50)
    print("错误：PyTorch 未安装。")
    print("="*50)
    exit(1)
# --- 结束 PyTorch 导入 ---
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
        # --- 【PyTorch】1. 初始化设备 ---
        if not torch.cuda.is_available():
            self.get_logger().fatal("PyTorch CUDA 不可用！GPU 加速无法启动。")
            self.destroy_node()
            rclpy.shutdown()
            return
        self.device = torch.device("cuda")
        self.get_logger().info(f"PyTorch 正在使用 GPU 设备: {self.device}")

        # --- 2. 声明所有参数 ---
        # 通用参数
        base_parameters = [
            ('image_topic', '/camera/image_raw'),
            ('pointcloud_topic', '/bev/obstacles'),
            ('bev_debug_image_topic', '/bev/debug_image'),
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
                base_parameters.append((f'{name}_{i}', [0,0,0]))
        self.declare_parameters(namespace='', parameters=base_parameters)

        # --- 3. 获取通用参数 ---
        self.enable_vis = self.get_parameter('enable_vis').value

        # 话题
        self.image_topic = self.get_parameter('image_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.bev_debug_image_topic = self.get_parameter('bev_debug_image_topic').value
        self.to_cmd_vel_topic = self.get_parameter('to_cmd_vel_topic').value

        # BEV 尺寸
        self.bev_width = self.get_parameter('bev_width').value
        self.bev_height = self.get_parameter('bev_height').value
        self.bev_size = (self.bev_width, self.bev_height)

        # 物理尺寸
        world_width_m_1 = self.get_parameter('world_width_m_1').value
        world_height_m_1 = self.get_parameter('world_height_m_1').value
        world_width_m_2 = self.get_parameter('world_width_m_2').value
        world_height_m_2 = self.get_parameter('world_height_m_2').value
        
        # 坐标系
        self.origin_offset_x_m_1 = self.get_parameter('origin_offset_x_m_1').value
        self.origin_offset_y_m_1 = self.get_parameter('origin_offset_y_m_1').value
        self.origin_offset_x_m_2 = self.get_parameter('origin_offset_x_m_2').value
        self.origin_offset_y_m_2 = self.get_parameter('origin_offset_y_m_2').value
                
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
        
        # --- 3. 加载两套 IPM 配置 ---
        self.ipm_configs = []
        try:
            self.ipm_configs.append(IPMConfig(self, 1))
            self.ipm_configs.append(IPMConfig(self, 2))
        except Exception as e:
            self.get_logger().error(f"加载 IPM 配置失败: {e}. 请检查你的 YAML 文件。")
            return


        # --- 4. 【PyTorch】加载并上传资源到 GPU ---
        self.get_logger().info("正在将资源上传到 GPU (PyTorch)...")
        try:
            # --- 替换 cv2.undistort ---
            # 1. 在 CPU 上计算一次重映射网格
            # (假设输入图像尺寸为 640x480)
            img_h, img_w = 480, 640 
            map1_cpu, map2_cpu = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs, None, self.camera_matrix, 
                (img_w, img_h), cv2.CV_32FC1
            )
            # 2. 创建 PyTorch grid_sample 网格
            self.grid_undistort = self.create_grid_from_cv_maps(map1_cpu, map2_cpu).to(self.device)

            # --- 替换 cv2.warpPerspective (H_1) ---
            src_pts_1 = np.float32(self.get_parameter('src_points_1').value).reshape(4, 2)
            dst_pts_1 = np.float32(self.get_parameter('dst_points_1').value).reshape(4, 2)
            self.bev_size_1 = (self.get_parameter('bev_height_1').value, self.get_parameter('bev_width_1').value)
            H_1_np = cv2.getPerspectiveTransform(src_pts_1, dst_pts_1)
            self.grid_warp_1 = self.create_grid_from_homography(H_1_np, self.bev_size_1).to(self.device)

            # --- 替换 cv2.warpPerspective (H_2) ---
            src_pts_2 = np.float32(self.get_parameter('src_points_2').value).reshape(4, 2)
            dst_pts_2 = np.float32(self.get_parameter('dst_points_2').value).reshape(4, 2)
            self.bev_size_2 = (self.get_parameter('bev_height_2').value, self.get_parameter('bev_width_2').value)
            H_2_np = cv2.getPerspectiveTransform(src_pts_2, dst_pts_2)
            self.grid_warp_2 = self.create_grid_from_homography(H_2_np, self.bev_size_2).to(self.device)

            # --- 物理参数 (在 CPU 上) ---
            self.meters_per_pixel_x_1 = self.get_parameter('world_width_m_1').value / self.bev_size_1[1]
            self.meters_per_pixel_y_1 = self.get_parameter('world_height_m_1').value / self.bev_size_1[0]
            self.origin_offset_x_m_1 = self.get_parameter('origin_offset_x_m_1').value
            self.origin_offset_y_m_1 = self.get_parameter('origin_offset_y_m_1').value
            
            self.meters_per_pixel_x_2 = self.get_parameter('world_width_m_2').value / self.bev_size_2[1]
            self.meters_per_pixel_y_2 = self.get_parameter('world_height_m_2').value / self.bev_size_2[0]
            self.origin_offset_x_m_2 = self.get_parameter('origin_offset_x_m_2').value
            self.origin_offset_y_m_2 = self.get_parameter('origin_offset_y_m_2').value
            
            # --- HSV 阈值 (上传到 GPU) ---
            # PyTorch HSV 范围: H (0-1), S (0-1), V (0-1)
            def convert_hsv_thresh(params_uint8):
                p = params_uint8
                # [H(0-180), S(0-255), V(0-255)] -> [H(0-1), S(0-1), V(0-1)]
                return torch.tensor([p[0] / 180.0, p[1] / 255.0, p[2] / 255.0], device=self.device, dtype=torch.float32)

            self.l_red1_1 = convert_hsv_thresh(self.get_parameter('hsv_lower_red1_1').value)
            self.u_red1_1 = convert_hsv_thresh(self.get_parameter('hsv_upper_red1_1').value)
            self.l_red2_1 = convert_hsv_thresh(self.get_parameter('hsv_lower_red2_1').value)
            self.u_red2_1 = convert_hsv_thresh(self.get_parameter('hsv_upper_red2_1').value)
            
            self.l_red1_2 = convert_hsv_thresh(self.get_parameter('hsv_lower_red1_2').value)
            self.u_red1_2 = convert_hsv_thresh(self.get_parameter('hsv_upper_red1_2').value)
            self.l_red2_2 = convert_hsv_thresh(self.get_parameter('hsv_lower_red2_2').value)
            self.u_red2_2 = convert_hsv_thresh(self.get_parameter('hsv_upper_red2_2').value)

            # --- 形态学内核 (上传到 GPU) ---
            self.morph_kernel_tensor = torch.from_numpy(self.morph_kernel).to(self.device).float().unsqueeze(0).unsqueeze(0)
            
            self.get_logger().info("GPU 资源初始化成功 (PyTorch)。")
            
        except Exception as e:
            self.get_logger().fatal(f"PyTorch GPU 初始化失败: {e}")
            self.destroy_node()
            rclpy.shutdown()
            return

        # --- 5. 初始化 ROS 接口 (使用 QoS) ---
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 15
        ) # !增加图像缓冲队列
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, self.pointcloud_topic, 5
        ) # !减少点云发布延迟，节省内存
        self.bev_image_1_pub = self.create_publisher(
            Image, self.bev_debug_image_topic, 10
        )
        self.to_cmd_vel_topic = self.create_publisher(
            Image, self.to_cmd_vel_topic, 10
        )
        
    def destroy_node(self):
        if self.enable_vis:
            cv2.destroyAllWindows()
        super().destroy_node()

    # --- 【PyTorch】辅助函数 ---
    
    def create_grid_from_cv_maps(self, map1: np.ndarray, map2: np.ndarray) -> torch.Tensor:
        """将 OpenCV remap 网格 (map1, map2) 转换为 F.grid_sample 网格 ([-1, 1])"""
        H, W = map1.shape
        # F.grid_sample 需要 (x, y) 格式
        grid_xy = torch.stack((torch.from_numpy(map1), torch.from_numpy(map2)), dim=-1) # (H, W, 2)
        
        # 归一化到 [-1, 1]
        grid_xy[..., 0] = (grid_xy[..., 0] / (W - 1)) * 2 - 1
        grid_xy[..., 1] = (grid_xy[..., 1] / (H - 1)) * 2 - 1
        
        return grid_xy.unsqueeze(0) # (1, H, W, 2)

    def create_grid_from_homography(self, H: np.ndarray, size: tuple) -> torch.Tensor:
        """根据单应性矩阵 H 为 F.grid_sample 创建网格"""
        H_inv = torch.inverse(torch.tensor(H, dtype=torch.float32))
        
        H_out, W_out = size
        # 创建目标网格
        y_coords, x_coords = torch.meshgrid(
            torch.linspace(0, H_out - 1, H_out),
            torch.linspace(0, W_out - 1, W_out),
            indexing='ij'
        )
        # (x, y, 1)
        grid = torch.stack([x_coords, y_coords, torch.ones_like(x_coords)], dim=-1).view(-1, 3)
        
        # 应用逆变换 H_inv * grid
        grid_src = torch.matmul(grid, H_inv.T)
        
        # 归一化 (x/z, y/z)
        grid_src = grid_src[:, :2] / grid_src[:, 2:]
        
        # 归一化到 [-1, 1] (假设原始图像尺寸为 640x480)
        H_in, W_in = 480, 640 
        grid_src[..., 0] = (grid_src[..., 0] / (W_in - 1)) * 2 - 1
        grid_src[..., 1] = (grid_src[..., 1] / (H_in - 1)) * 2 - 1
        
        return grid_src.view(H_out, W_out, 2).unsqueeze(0) # (1, H, W, 2)

    def bgr_to_hsv_torch(self, img_bgr_float: torch.Tensor) -> torch.Tensor:
        """在 PyTorch (GPU) 上将 BGR (0-1) 转换为 HSV (H:0-1, S:0-1, V:0-1)"""
        # BGR (0-1) -> RGB (0-1)
        img_rgb = img_bgr_float.flip(1) # [B, C, H, W]
        B, C, H, W = img_rgb.shape
        
        r, g, b = img_rgb[:, 0, :, :], img_rgb[:, 1, :, :], img_rgb[:, 2, :, :]
        
        max_val, _ = torch.max(img_rgb, dim=1) # Value
        min_val, _ = torch.min(img_rgb, dim=1)
        delta = max_val - min_val
        
        # Hue
        hue = torch.zeros_like(max_val)
        mask_r = (max_val == r) & (delta != 0)
        mask_g = (max_val == g) & (delta != 0)
        mask_b = (max_val == b) & (delta != 0)
        
        hue[mask_r] = (g[mask_r] - b[mask_r]) / delta[mask_r]
        hue[mask_g] = 2.0 + (b[mask_g] - r[mask_g]) / delta[mask_g]
        hue[mask_b] = 4.0 + (r[mask_g] - g[mask_g]) / delta[mask_g]
        
        hue = (hue / 6.0) % 1.0 # 归一化到 [0, 1]
        
        # Saturation
        saturation = torch.where(max_val > 1e-6, delta / max_val, 0.0)
        
        # Value (已计算)
        
        return torch.stack([hue, saturation, max_val], dim=1) # [B, C, H, W]

    def in_range_torch(self, img_hsv: torch.Tensor, lower: torch.Tensor, upper: torch.Tensor) -> torch.Tensor:
        """PyTorch 版 inRange"""
        mask = (img_hsv[:, 0, :, :] >= lower[0]) & (img_hsv[:, 0, :, :] <= upper[0]) & \
               (img_hsv[:, 1, :, :] >= lower[1]) & (img_hsv[:, 1, :, :] <= upper[1]) & \
               (img_hsv[:, 2, :, :] >= lower[2]) & (img_hsv[:, 2, :, :] <= upper[2])
        return mask # [B, H, W]

    def morph_torch(self, image, kernel, mode='opening'):
        """PyTorch 版形态学操作 (Open 或 Close)"""
        # (B, 1, H, W)
        B, C, H, W = image.shape
        k_H, k_W = kernel.shape[-2:]
        padding = (k_H // 2, k_W // 2)
        
        # Dilation = max_pool
        # Erosion = -max_pool(-image)
        if mode == 'opening':
            # Erosion -> Dilation
            eroded = -F.max_pool2d(-image, kernel_size=(k_H, k_W), stride=1, padding=padding)
            opened = F.max_pool2d(eroded, kernel_size=(k_H, k_W), stride=1, padding=padding)
            return opened
        elif mode == 'closing':
            # Dilation -> Erosion
            dilated = F.max_pool2d(image, kernel_size=(k_H, k_W), stride=1, padding=padding)
            closed = -F.max_pool2d(-dilated, kernel_size=(k_H, k_W), stride=1, padding=padding)
            return closed
        return image

    # --- 核心回调函数 ---
    
    def image_callback(self, msg: Image):
        try:
            # (CPU) 1. Bridge
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge 转换失败: {e}')
            return
            
        # (CPU -> GPU) 2. 上传图像 (B, C, H, W), float, 0-1
        img_tensor_float = torch.from_numpy(cv_image).to(self.device, non_blocking=True).float().div(255.0).permute(2, 0, 1).unsqueeze(0)

        # --- 【GPU】3. 畸变校正 ---
        undistorted_gpu = F.grid_sample(img_tensor_float, self.grid_undistort, mode='bilinear', padding_mode='zeros', align_corners=False)

        # --- 【GPU】4. 逆透视变换 (IPM) x2 ---
        bev_gpu_1 = F.grid_sample(undistorted_gpu, self.grid_warp_1, mode='bilinear', padding_mode='zeros', align_corners=False)
        bev_gpu_2 = F.grid_sample(undistorted_gpu, self.grid_warp_2, mode='bilinear', padding_mode='zeros', align_corners=False)

        # --- 【GPU】5. 颜色分割 x2 ---
        hsv_gpu_1 = self.bgr_to_hsv_torch(bev_gpu_1)
        hsv_gpu_2 = self.bgr_to_hsv_torch(bev_gpu_2)
        
        # BEV 1 Mask (H:0-1, S:0-1, V:0-1)
        mask1_1 = self.in_range_torch(hsv_gpu_1, self.l_red1_1, self.u_red1_1)
        mask2_1 = self.in_range_torch(hsv_gpu_1, self.l_red2_1, self.u_red2_1)
        red_mask_1 = (mask1_1 | mask2_1)

        # BEV 2 Mask
        mask1_2 = self.in_range_torch(hsv_gpu_2, self.l_red1_2, self.u_red1_2)
        mask2_2 = self.in_range_torch(hsv_gpu_2, self.l_red2_2, self.u_red2_2)
        red_mask_2 = (mask1_2 | mask2_2)
        
        # --- 【GPU】6. 形态学操作 x2 ---
        # (B, H, W) -> (B, 1, H, W)
        red_mask_1 = red_mask_1.unsqueeze(1).float()
        red_mask_2 = red_mask_2.unsqueeze(1).float()
        
        red_mask_1 = self.morph_torch(red_mask_1, self.morph_kernel_tensor, mode='opening')
        red_mask_1 = self.morph_torch(red_mask_1, self.morph_kernel_tensor, mode='closing')
        
        red_mask_2 = self.morph_torch(red_mask_2, self.morph_kernel_tensor, mode='opening')
        red_mask_2 = self.morph_torch(red_mask_2, self.morph_kernel_tensor, mode='closing')

        # --- 【GPU -> CPU】7. 下载结果 ---
        torch.cuda.synchronize() # 确保所有 GPU 操作完成
        
        # 转换为 uint8 NumPy 数组
        mask_np_1 = red_mask_1.squeeze().byte().cpu().numpy()
        mask_np_2 = red_mask_2.squeeze().byte().cpu().numpy()

        # --- 【CPU】8. 查找障碍物像素 ---
        obstacle_pixels_1 = cv2.findNonZero(mask_np_1)
        obstacle_pixels_2 = cv2.findNonZero(mask_np_2)

        # --- 【CPU】9. 转换为 PointCloud2 ---
        points_list = []
        if obstacle_pixels_1 is not None:
            for point in obstacle_pixels_1:
                u, v = point[0]
                robot_x = self.origin_offset_x_m_1 + (self.bev_size_1[0] - v) * self.meters_per_pixel_y_1
                robot_y = self.origin_offset_y_m_1 - u * self.meters_per_pixel_x_1
                points_list.append([robot_x, robot_y, 0.0]) # Z=0.0 (X, Y, Z)

        if obstacle_pixels_2 is not None:
            for point in obstacle_pixels_2:
                u, v = point[0]
                robot_x = self.origin_offset_x_m_2 + (self.bev_size_2[0] - v) * self.meters_per_pixel_y_2
                robot_y = self.origin_offset_y_m_2 - u * self.meters_per_pixel_x_2
                points_list.append([robot_x, robot_y, 0.0])

        # --- 【CPU】10. 创建并发布点云 ---
        header = Header(stamp=msg.header.stamp, frame_id="body")
        
        # fields = [
        #     PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        # ]
        # point_cloud_msg = point_cloud2.create_cloud(header, fields, points_list)
        # self.pointcloud_pub.publish(point_cloud_msg)
            
        # --- 【CPU】11. 可选的可视化和调试图像发布 ---
        if self.enable_vis or self.bev_image_pub.get_subscription_count() > 0:
            # (GPU -> CPU) 只有在需要时才下载 BEV 图像
            bev_image_1_np = (bev_gpu_1.squeeze().permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
            bev_image_2_np = (bev_gpu_2.squeeze().permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
            
            # 发布调试图像 (Config 1)
            debug_bev_display_1 = cv2.bitwise_and(bev_image_1_np, bev_image_1_np, mask=mask_np_1)
            bev_img_msg = self.bridge.cv2_to_imgmsg(debug_bev_display_1, "bgr8")
            bev_img_msg.header = header
            self.bev_image_pub.publish(bev_img_msg)

            if self.enable_vis:
                # (GPU -> CPU) 下载原始图像
                undistorted_img_np = (undistorted_gpu.squeeze().permute(1, 2, 0).cpu().numpy() * 255.0).astype(np.uint8)
                vis_img = undistorted_img_np.copy()
                
                src_pts_1 = np.float32(self.get_parameter('src_points_1').value).reshape(4, 2)
                src_pts_2 = np.float32(self.get_parameter('src_points_2').value).reshape(4, 2)
                for pt in src_pts_1:
                         cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 5, (0, 255, 0), -1)
                for pt in src_pts_2:
                         cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
                cv2.imshow("1. Undistorted Image (PyTorch)", vis_img)
                
                # 显示 BEV 结果
                debug_bev_display_2 = cv2.bitwise_and(bev_image_2_np, bev_image_2_np, mask=mask_np_2)
                cv2.imshow("2.1 BEV Result (Config 1)", debug_bev_display_1)
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