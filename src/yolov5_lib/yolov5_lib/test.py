import yaml
import time
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
# from yolo import YOLO
from models.common import DetectMultiBackend
from utils.general import non_max_suppression
import torch
from torchvision import transforms
from ros_yolo_detect import DetectorYolov5
import matplotlib.pyplot as plt


def img_process(img_pil, device, target_size=(640, 640)):
    w, h = img_pil.size
    if w == h:
        padded_img = img_pil
    else:
        dim_to_pad = 1 if w < h else 2
        if dim_to_pad == 1:
            padding = (h - w) / 2
            padded_img = PILImage.new('RGB', (h, h), color=(127, 127, 127))
            padded_img.paste(img_pil, (int(padding), 0))

        else:
            padding = (w - h) / 2
            padded_img = PILImage.new('RGB', (w, w), color=(127, 127, 127))
            padded_img.paste(img_pil, (0, int(padding)))

    resize = transforms.Resize((target_size[0], target_size[1]))
    padded_img = resize(padded_img)  # choose here
    transform = transforms.ToTensor()
    img_tensor = transform(padded_img).unsqueeze(0)
    if device == "cuda":
        # print('XXXXXX')
        img_tensor = img_tensor.to(device)
    return img_tensor


img_path = f"./1.png"
img_pil = PILImage.open(img_path).convert('RGB')
# trans_2pilimage = transforms.ToPILImage()
# img_pil_pad = trans_2pilimage(img_process(img_pil, 'cpu')[0].cpu())
# detector = DetectorYolov5(cfgfile="./data/coco.yaml",
#                  weightfile="./yolov5l.pt")
detector = DetectorYolov5(cfgfile="./inCar_files/carDetect.yaml",
                 weightfile="./inCar_files/best.pt")
target_lab_name = 'vehicle'

result_image, bbox = detector.detect(img_pil)
bbox = bbox.cpu().detach().numpy()

top_label = bbox[:, 5]
top_conf = bbox[:, 4]
top_boxes = bbox[:, :4]

# 只保留 person 类别
if target_lab_name not in detector.yaml["names"].values():
    print(f"{target_lab_name} 类别不在 class_names 中！")
    

target_class_id = detector.name2id(target_lab_name)
target_class_mask = top_label == target_class_id

if not np.any(target_class_mask):
    print("No traget obj detected, skipping publish.")
    

top_label = top_label[target_class_mask]
top_conf = top_conf[target_class_mask]
top_boxes = top_boxes[target_class_mask]


input_imgs = img_process(img_pil, 'cuda', (640, 640))
detections = detector.model(input_imgs)[0]
print(bbox)
print(detections.size())
print(detections)
result_image = cv2.cvtColor(np.asarray(result_image), cv2.COLOR_RGB2BGR)
cv2.imshow("YOLO Detection", result_image)
cv2.waitKey(0)