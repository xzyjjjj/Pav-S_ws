import os
import cv2
from cv_bridge import CvBridge
import rosbag2_py

bag_path = "/Pav-S_ws/src/costmap_process/new/debug/rgb_img_bag"
out_dir = "/Pav-S_ws/src/costmap_process/new/debug"
os.makedirs(out_dir, exist_ok=True)
bridge = CvBridge()

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)

count = 0
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == "/rgb_img":
        from sensor_msgs.msg import Image
        import rclpy.serialization
        msg = rclpy.serialization.deserialize_message(data, Image)
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        fname = os.path.join(out_dir, f"rgb_img_{count:05d}.png")
        cv2.imwrite(fname, img)
        print(f"Saved {fname}")
        count += 1