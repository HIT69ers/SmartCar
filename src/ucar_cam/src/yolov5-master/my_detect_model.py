# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
import os
import sys
from pathlib import Path

import torch

FILE = Path(__file__).resolve()  # 查找detect.py文件的绝对路径
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.dataloaders import my_LoadImages, my_loadimg_single
from utils.general import check_img_size, non_max_suppression, scale_boxes
from utils.torch_utils import select_device


def myLoadModel(
        weights=ROOT / 'yolov5s.pt',  # model path or triton URL
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
):
    device = select_device(device)  # 选择设备
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, pt = model.stride, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size
    model.warmup(imgsz=(1 if pt or model.triton else 1, 3, *imgsz))  # 热身
    return model, imgsz, pt


def myPredict(model, source, imgsz, pt, stride, conf_thres=0.25,  # confidence threshold
              iou_thres=0.45,  # NMS IOU threshold
              max_det=1000,
              classes=None,
              agnostic_nms=False, ):
    dataset = my_LoadImages(source, img_size=imgsz, stride=stride, auto=pt)  # 加载数据

    # im0为原图 im为处理后的图
    detc = []
    for im, im0s in dataset:
        im = torch.from_numpy(im).to(model.device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        pred = model(im, augment=False, visualize=False)  # 1,18900,85

        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms,  # 非极大值抑制
                                   max_det=max_det)  # [1,5,6] 5个检测框 6个预测信息

        for i, det in enumerate(pred):
            if len(det):
                # 将预测框映射到原图上
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0s.shape).round()  # 坐标即是原图的坐标
                detc.append(det)  # 三个维度的
    return detc


def myPredict_single(model, im0, imgsz, pt, stride, conf_thres=0.25,  # confidence threshold
                     iou_thres=0.45,  # NMS IOU threshold
                     max_det=1000,
                     classes=None,
                     agnostic_nms=False):
    im = my_loadimg_single(im0, img_size=imgsz, stride=stride, auto=pt)
    im = torch.from_numpy(im).to(model.device)
    im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
    im /= 255  # 0 - 255 to 0.0 - 1.0
    if len(im.shape) == 3:
        im = im[None]  # expand for batch dim #增加一个维度，batch维度

    pred = model(im, augment=False, visualize=False)  # 1,18900,85 #经过模型后预测

    # NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms,  # 非极大值抑制
                               max_det=max_det)  # [1,5,6] 5个检测框 6个预测信息

    detc = []
    for i, det in enumerate(pred):
        if len(det):
            # 将预测框映射到原图上
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # 坐标即是原图的坐标
            detc.append(det)  # 三个维度的
    return detc

