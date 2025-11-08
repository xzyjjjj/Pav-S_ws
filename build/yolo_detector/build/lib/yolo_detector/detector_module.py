#!/usr/bin/env python3
# coding=utf-8
import yaml
import time
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
# from yolo import YOLO
from models.common import DetectMultiBackend
from utils.general import non_max_suppression
import torch
from torchvision import transforms


class DetectorYolov5():
    def __init__(self,
                 cfgfile="/yolov5/data/carDetect.yaml",
                 weightfile="/yolov5/runs/train/car_yolov5s/weights/best.pt",
                 target_size=(640, 640)):

        with open(cfgfile, "r") as f:
            self.yaml = yaml.safe_load(f)

        # Set up model
        self.model = DetectMultiBackend(weights=weightfile, data=cfgfile, dnn=False, fp16=False)
        # self.model.float()
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt

        # self.model.eval().cuda()
        self.img_size = target_size

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        if self.device == "cuda":
            # # 打印所使用的 GPU 信息
            # print(f"✅ Detector initialized using: {self.device} ({torch.cuda.get_device_name(0)})\n")
            self.use_cuda = True
            self.model.to(self.device)

        else:
            # print(f"❌ Detector initialized using: {self.device} (CUDA not available)\n")
            # print(f"Use cpu instead.\n")
            self.use_cuda = False

    def id2name(self, idx):
        name = None
        for k, v in self.yaml["names"].items():
            if k == int(idx):
                name = v
                break
        return name

    def name2id(self, name):
        idx = None
        for k, v in self.yaml["names"].items():
            if v == name:
                idx = k
                break
        return idx

    # 检测函数
    def detect(self, img_pil):
        # resize image
        orig_size = img_pil.size
        input_imgs = img_process(img_pil, self.device, self.img_size)

        # Get detections
        self.model.eval()
        detections = self.model(input_imgs)[0]  ## v5:torch.Size([8, 25200, 85])
        if not (detections[0] == None):
            # init cls_id_attacked
            # 执行非极大值抑制 (NMS)，消除对同一物体的多个重叠框检测
            bboxes = non_max_suppression(detections, conf_thres=0.5, iou_thres=0.4)  ## <class 'list'>.
            # print(bboxes)
            # only non None. Replace None with torch.tensor([])
            bboxes = [torch.tensor([]) if bbox is None else bbox for bbox in bboxes]
            # 将 NMS 输出的边界框坐标（在填充和缩放后的图像上的坐标）转换回原始图像尺寸下的坐标，
            # 并从 (left, top, right, bottom) 格式转换为 (center_x, center_y, width, height) 格式
            bboxes = [bboxes_to_yolo_format(bbox, orig_size, self.img_size) if bbox.dim() == 2 else bbox for bbox in
                      bboxes]
        else:
            bboxes = []
        
        bbox = bboxes[0].cpu()
        # draw img with bbox
        pil = img_pil.copy()
        img_width = pil.size[0]
        img_height = pil.size[1]

        for box in bbox:
            class_id = int(box[5])
            conf = float(box[4])

            x_center = box[0]
            y_center = box[1]
            width = box[2]
            height = box[3]
            # left = int((x_center.item() - width.item() / 2) * img_width)
            # right = int((x_center.item() + width.item() / 2) * img_width)
            # top = int((y_center.item() - height.item() / 2) * img_height)
            # bottom = int((y_center.item() + height.item() / 2) * img_height)
            left = int((x_center.item() - width.item() / 2))
            right = int((x_center.item() + width.item() / 2))
            top = int((y_center.item() - height.item() / 2))
            bottom = int((y_center.item() + height.item() / 2))

            # img with prediction
            draw = ImageDraw.Draw(pil)
            shape = [left, top, right, bottom]
            draw.rectangle(shape, outline="red")
            # text
            color = [255, 0, 0]
            font = ImageFont.load_default(size=int(min(img_width, img_height) / 18))
            sentence = str(self.id2name(class_id)) + " (" + str(round(float(conf), 2)) + ")"
            position = [left, top]
            draw.text(tuple(position), sentence, tuple(color), font=font)

        return pil, bbox


def img_process(img_pil, device, target_size=(640, 640)):
    w, h = img_pil.size
    # 在较短的维度填充灰色（127, 127, 127）像素，将原始图像变成一个正方形
    if w == h:
        padded_img = img_pil
    else:
        dim_to_pad = 1 if w < h else 2
        if dim_to_pad == 1:
            padding = (h - w) / 2
            padded_img = PILImage.new('RGB', (h, h), color=(127, 127, 127)) # 创建一个正方形灰色背景
            padded_img.paste(img_pil, (int(padding), 0)) # 原图放在中间

        else:
            padding = (w - h) / 2
            padded_img = PILImage.new('RGB', (w, w), color=(127, 127, 127))
            padded_img.paste(img_pil, (0, int(padding)))

    #  将填充后的正方形图像缩放到模型需要的目标尺寸
    resize = transforms.Resize((target_size[0], target_size[1]))
    padded_img = resize(padded_img)  

    transform = transforms.ToTensor()
    img_tensor = transform(padded_img).unsqueeze(0) # 添加批量维度

    if device == "cuda":
        # print('XXXXXX')
        img_tensor = img_tensor.to(device)
    return img_tensor

# 将模型在预处理后图像（填充并缩放过的 640x640 图像）上检测到的边界框坐标，转换回原始图像坐标系下的坐标。
def bboxes_to_yolo_format(bbox_tensor, orig_size, final_size):
    """
    bbox_tensor: (N, 6), each row: (left, top, right, bottom, conf, cls_id)
    orig_size: (orig_w, orig_h) - original image size
    final_size: (w2, h2) - resized (padded) input size to detector
    return: (N, 6), each row: (cx, cy, w, h, conf, cls_id) in YOLO format, normalized
    """

    orig_w, orig_h = orig_size
    w2, h2 = final_size

    orig_ratio = orig_w / orig_h
    target_ratio = w2 / h2

    if orig_ratio > target_ratio: # padding时是 长度填充到和宽度一致
        scale = w2 / orig_w
        new_h = orig_h * scale
        pad_top = (h2 - new_h) / 2
        pad_left = 0
    else: # padding时是 宽度填充到和长度一致
        scale = h2 / orig_h # 小正方形放大为大正方形的缩放比例
        new_w = orig_w * scale # 原始有效宽度被放大后的宽度（即大正方形去掉padding后的宽度）
        pad_left = (w2 - new_w) / 2 # 左边填充区域的最大横坐标
        pad_top = 0

    # Remove padding and rescale back to original
    # 将大正方形中的检测框坐标 放缩回原始图片的对应坐标
    left = (bbox_tensor[:, 0] - pad_left) / scale   
    top = (bbox_tensor[:, 1] - pad_top) / scale
    right = (bbox_tensor[:, 2] - pad_left) / scale
    bottom = (bbox_tensor[:, 3] - pad_top) / scale

    # Clip to valid image range
    # 有效坐标截断
    left = left.clamp(0, orig_w)
    right = right.clamp(0, orig_w)
    top = top.clamp(0, orig_h)
    bottom = bottom.clamp(0, orig_h)

    # Convert to YOLO format (normalized)
    cx = (left + right) / 2 #/ orig_w
    cy = (top + bottom) / 2 #/ orig_h
    bw = (right - left) #/ orig_w
    bh = (bottom - top) #/ orig_h

    conf = bbox_tensor[:, 4]
    cls_id = bbox_tensor[:, 5]

    yolo_tensor = torch.stack([cx, cy, bw, bh, conf, cls_id], dim=1)
    # yolo_tensor = torch.stack([top, left, bottom, right, conf, cls_id], dim=1)
    return yolo_tensor
