#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Trace:
    def __init__(self, detect):
        self.detect = detect  # 拍照用的

        # 创建cv_bridge实例
        self.bridge = CvBridge()

        # 创建发布者，发布到'image_topic'话题
        self.image_pub = rospy.Publisher('image_topic', Image, queue_size=10)

        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))  # 腐蚀核

        # 设置循环的频率
        self.rate = rospy.Rate(10)
        # self.take_index = 0
        self.thre_low = detect.config['thre_low']
        self.thre_high = detect.config['thre_high']

    def main(self):
        self.detect.get_pic(mode=3)  # 巡线拍摄,也用2方法进行拍摄

        while rospy.get_param("stop_trace", 0) == 0:
            self.detect.set_photo_flag(1)  # 允许拍摄
            self.detect.event.wait()

            # self.take_index += 1
            # cv2.imwrite(f"/home/ucar/ucar_ws/src/ucar_cam/src/yolov5-master/data/my_imgs/imgs_{self.take_index}.jpg", self.detect.single_img)

            img = cv2.cvtColor(self.detect.single_img, cv2.COLOR_BGR2GRAY)  # 转换为灰度图
            img = cv2.GaussianBlur(img, (5, 5), 0)  # 高斯滤波

            edge = cv2.Canny(img, 90,180 , L2gradient=True)  # Canny算子检测边缘 140/180
            edge = 255 - edge  # 取反

            edge = cv2.erode(edge, self.kernel, iterations=2)  # 腐蚀
            pub_img = cv2.resize(edge, (188, 120))  # 减小尺寸

            # 将OpenCV图像转换为ROS Image消息 mono8表示额单通道的8位图像编码方式
            # image_message = self.bridge.cv2_to_imgmsg(pub_img, "mono8")
            image_message = self.bridge.cv2_to_imgmsg(pub_img, "mono8")

            # 发布图像
            self.image_pub.publish(image_message)
            print("pub img")                                         

            # 等待一段时间
            self.rate.sleep()

        self.detect.set_photo_flag(2)  # 关闭拍摄


if __name__ == "__main__":
    import yaml
    import os
    import sys
    
    folder_path = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(f"{folder_path}/../src/yolov5-master/")  # 添加yolo文件夹的路径
    
    from my_detect import Detect

    rospy.init_node("test_trace")  # 节点创建
    folder_path = os.path.dirname(os.path.abspath(__file__)) #当前文件夹

    with open(f'{folder_path}/my_params/rec_params.yml', 'r') as f:  # 识别参数
        config = yaml.load(f, Loader=yaml.FullLoader)  # 全部读取为字典
    with open(f'{folder_path}/my_params/classes.yml', 'r') as f:  # 类别参数
        class_config = yaml.load(f, Loader=yaml.FullLoader)
   
    detect = Detect(config, class_list=class_config['classes'], test_mode=1)  # 获取yolo探测器, 输入摄像头参数和类别列表信息

    trace = Trace(detect)  # 巡线类
    trace.main()  # 巡线主函数
