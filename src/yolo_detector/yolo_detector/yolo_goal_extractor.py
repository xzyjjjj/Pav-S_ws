import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.parameter import Parameter

# 消息类型
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped

# TF2 库
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs # 用于转换 PoseStamped

class YoloGoalExtractor(Node):

    def __init__(self):
        super().__init__('yolo_goal_extractor')

        # --- 1. 声明和获取参数 ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('detection_topic', '/yolo_detections'),
                ('bonus_goal_topic', '/bonus_goals'),
                ('robot_base_frame', 'body'),
                ('global_frame', 'map'),
                ('tf_timeout_seconds', 0.1),
                ('bev_width', 640),
                ('bev_height', 480),
                ('world_width_m', 2.0),
                ('world_height_m', 3.0),
                ('origin_offset_x_m', 0.5),
                ('origin_offset_y_m', 1.0)
            ]
        )
        
        # 获取话题
        detection_topic = self.get_parameter('detection_topic').value
        self.bonus_goal_topic = self.get_parameter('bonus_goal_topic').value
        
        # 获取坐标系
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.tf_timeout_seconds = self.get_parameter('tf_timeout_seconds').value
        
        # 获取 IPM 参数
        self.bev_width = self.get_parameter('bev_width').value
        self.bev_height = self.get_parameter('bev_height').value
        self.origin_offset_x_m = self.get_parameter('origin_offset_x_m').value
        self.origin_offset_y_m = self.get_parameter('origin_offset_y_m').value

        # 计算物理比例
        world_width_m = self.get_parameter('world_width_m').value
        world_height_m = self.get_parameter('world_height_m').value
        self.meters_per_pixel_x = world_width_m / self.bev_width
        self.meters_per_pixel_y = world_height_m / self.bev_height

        # --- 2. 初始化 TF2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- 3. 初始化 ROS 接口 ---
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            detection_topic,
            self.detection_callback,
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.bonus_goal_topic,
            10
        )
        
        self.get_logger().info("YOLO Goal Extractor 已启动。")

    def detection_callback(self, msg: Detection2DArray):
        
        if not msg.detections:
            self.publish_empty_markers(msg.header.stamp)
            return

        # (修改点 #2) 找到当前帧中价值最高的检测
        best_detection = None
        max_value = -1

        for det in msg.detections:
            try:
                # class_id 是 'num_1', 'num_2', ..., 'num_8'
                # 访问路径：Detection2D -> ObjectHypothesisWithPose -> ObjectHypothesis -> class_id
                # self.get_logger().info(f"class_id:{det.results[0].hypothesis.class_id[4]}\n")
                value = int(det.results[0].hypothesis.class_id[4])
                if value > max_value:
                    max_value = value
                    best_detection = det
            except (ValueError, IndexError, AttributeError) as e:
                self.get_logger().warn(f"无法解析检测ID: {e}")
                continue

        if best_detection is None:
            self.publish_empty_markers(msg.header.stamp)
            return
            
        # 1. 像素坐标 -> body 坐标系
        u = best_detection.bbox.center.position.x
        v = best_detection.bbox.center.position.y
        
        robot_x = self.origin_offset_x_m + (self.bev_height - v) * self.meters_per_pixel_y
        robot_y = self.origin_offset_y_m - u * self.meters_per_pixel_x

        # 2. 创建 'body' 坐标系下的 PoseStamped
        pose_in_body = PoseStamped()
        pose_in_body.header.frame_id = self.robot_base_frame

        # 假设 50 毫秒的滞后足以保证 TF 数据已到达
        stamp_rclpy = Time.from_msg(msg.header.stamp)
        safe_stamp_rclpy = stamp_rclpy - Duration(seconds=0.1)
        safe_stamp_msg = safe_stamp_rclpy.to_msg()
        pose_in_body.header.stamp = safe_stamp_msg # 替换为安全时间戳
        # pose_in_body.header.stamp = msg.header.stamp

        pose_in_body.pose.position.x = robot_x
        pose_in_body.pose.position.y = robot_y
        pose_in_body.pose.orientation.w = 1.0
        # 3. (修改点 #1) 变换到 'map' 坐标系
        try:
            pose_in_map = self.tf_buffer.transform(
                pose_in_body,
                self.global_frame,
                timeout=Duration(seconds=self.tf_timeout_seconds)
            )
        except TransformException as e:
            self.get_logger().warn(f"TF 变换失败 ({self.robot_base_frame} -> {self.global_frame}): {e}")
            return

        # 4. (修改点 #3) 创建 MarkerArray
        marker_array = MarkerArray()
        
        # --- 创建一个 Marker 来代表这个目标 ---
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bonus_goals"
        
        # 将数字值 (得分) 编码到 marker.id 中
        marker.id = max_value 
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_in_map.pose
        
        # 设置可视化属性 (例如，一个半透明的绿色球体)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 0.8
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        # Marker 的生命周期（例如 1.0 秒），确保它不会永久停留
        marker.lifetime = Duration(seconds=1.0).to_msg()
        
        marker_array.markers.append(marker)
        
        # --- 创建一个 Marker 来显示数字值 (可选但推荐) ---
        text_marker = Marker()
        text_marker.header.frame_id = self.global_frame
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "bonus_labels"
        text_marker.id = max_value # 保持 ID 一致
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose = pose_in_map.pose
        text_marker.pose.position.z += 0.5 # 让文字浮在球体上方
        text_marker.text = str(max_value)
        text_marker.scale.z = 0.5 # 文字大小
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.lifetime = Duration(seconds=1.0).to_msg()

        marker_array.markers.append(text_marker)

        # 5. 发布
        self.marker_pub.publish(marker_array)


    def publish_empty_markers(self, stamp):
        """发布一个空的 MarkerArray """
        marker_array = MarkerArray()
        
        # # 创建一个 DELETEALL 标记
        # marker = Marker()
        # marker.header.stamp = self.get_clock().now().to_msg()
        # marker.ns = "bonus_goals"
        # marker.action = Marker.DELETEALL
        
        # text_marker = Marker()
        # text_marker.header.stamp = self.get_clock().now().to_msg()
        # text_marker.ns = "bonus_labels"
        # text_marker.action = Marker.DELETEALL
        
        # marker_array.markers.append(marker)
        # marker_array.markers.append(text_marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = YoloGoalExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()