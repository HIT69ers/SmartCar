#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import threading
from threading import Thread
import platform
import pathlib
import yaml
import rospy

from rknnlite_detect_model import rknnLoadModel, rknnPredict_single
from my_utils import free_dir, drawtext
from my_ResultFilter import FilterResult

import os
folder_path = os.path.dirname(os.path.abspath(__file__))

plt = platform.system()
if plt != 'Windows':
    pathlib.WindowsPath = pathlib.PosixPath


# 新建一个对象 调用getPic就可以获得一个列表
class Detect:
    """
    检测类
    负责摄像头管理
    网络推理（不包括处理推理结果）
    多线程操作 ---- 子线程是摄像头线程， 主线程控制网络推理
    """

    def __init__(self, config, class_list, test_mode=0):
        """
        初始化检测对象
        """
        self.isTake = 0  # 一个标志 0 是等待状态 1 是拍摄状态
        self.camera = 0  # 第几个摄像头
        self.cap = None  # 捕获视频的对象
        self.classes = []
        for key in class_list:
            self.classes.append(class_list[key])  # 识别的类别, 获得类别名称

        with open(f'{folder_path}/../../scripts/my_params/cap_params.yml', 'r') as f2:  # 摄像头参数
            self.param_list = yaml.load(f2, Loader=yaml.FullLoader)  # 摄像头全部的参数列表

        self.paramFlag = config["capParamFlag"]  # 摄像头选定参数
        self.width = 320  # 宽
        self.height = 240  # 高
        self.start_camera()  # 开启摄像头

        # take_photo_1 的保存路径
        self.take_path_1 = f"{folder_path}/data/savePath_test/frame.jpg"

        # 测试图片保存路径
        self.test_save_path = f"{folder_path}/data/test_img/"
        free_dir(self.test_save_path)

        self.saveIndex = 0  # 保存序号
        if test_mode == 0:
            self.imgsz, self.pt, self.rknn= rknnLoadModel(weights=f'{folder_path}/my_model/' + config['model_file'])

        self.event = threading.Event()  # 线程事件对象，用于线程同步和通信

        self.single_img = None  # 使用单次不保存读取时候，拍照线程和识别线程共享的图片变量

        self.result_filter = FilterResult(config, class_list)  # 结果过滤器
        self.config = config

    def start_camera(self):
        """
        开启/重启摄像头
        """
        recover_flag = False  # 是否是重启
        if self.cap:  # 如果存在
            self.release()
            recover_flag = True

        self.cap = cv2.VideoCapture(self.camera)  # 视频捕获对象

        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.param_list[self.paramFlag]['BRIGHTNESS'])  # 亮度参数
        self.cap.set(cv2.CAP_PROP_CONTRAST, self.param_list[self.paramFlag]['CONTRAST'])  # 对比度参数
        self.cap.set(cv2.CAP_PROP_SATURATION, self.param_list[self.paramFlag]['SATURATION'])  # 饱和度参数

        self.cap.set(3, self.width)  # 摄像头宽属性
        self.cap.set(4, self.height)  # 摄像头高属性

        codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')  # 视频格式属性
        self.cap.set(cv2.CAP_PROP_FOURCC, codec)

        if recover_flag:
            print(">>>> 重新加载成功")
        else:
            print(">>>> 首次启动成功")
        print("--------摄像头属性--------")
        print(f"cv2.CAP_PROP_BRIGHTNESS:{self.cap.get(cv2.CAP_PROP_BRIGHTNESS)}")
        print(f"cv2.CAP_PROP_CONTRAST:{self.cap.get(cv2.CAP_PROP_CONTRAST)}")
        print(f"cv2.CAP_PROP_SATURATION:{self.cap.get(cv2.CAP_PROP_SATURATION)}")
        print(codec)

    def get_pic(self, mode):
        """
        启动拍摄程序
        """
        p = Thread(target=self.select_tp, args=(mode,))  # 开一个循环拍照片的进程，输入的是图片的保存路径
        p.start()  # 开始进程
        return p

    def select_tp(self, mod):
        """
        不同模式的loopTakePhoto选择
        """
        if mod == 1:
            self.take_photo_1(self.take_path_1)  # 路径是定的
        elif mod == 2:
            self.take_photo_2()
        elif mod == 3:
            self.take_photo_3()
        elif mod == 4:
            self.take_photo_4()
        else:
            pass

    def take_photo_1(self, path):
        """
        1号方式循环拍照片
        保存拍摄方法
        每次拍照状态拍摄并保存一张照片
        """
        while not rospy.is_shutdown():  # 节点没被关闭
            if self.isTake == 1:  # isTake被设置为1才能够拍照，这样为了便于外部控制
                ret, frame = self.cap.read()  # 读取图像
                frame = cv2.flip(frame, 1)  ##图像左右颠倒

                if ret == False or frame is None:  # 如果没有图像
                    self.start_camera()  # 重新启动摄像头
                    self.take_photo_1(path)  # 重新进入这个函数
                    return  # 退出
                else:
                    cv2.imwrite(path, frame)
                    self.set_photo_flag(0)  # 设置为0，进入等待状态
                    self.event.set()  # 设置事件，保证拍照完了才进行推理

            elif self.isTake == 2:  # 摄像头释放
                self.release()  # 释放
                break

            cv2.waitKey(10)  # 进行一定的等待， 不用一只循环太多
        self.set_photo_flag(0)

    def take_photo_2(self):
        """
        2号拍摄方法
        不保存拍摄方法
        每次拍照状态拍摄一张照片，不进行保存
        """
        while not rospy.is_shutdown():  # 节点没关闭
            ret, self.single_img = self.cap.read()  # 读取图像
            if self.isTake == 1:  # 拍照状态
                self.single_img = cv2.flip(self.single_img, 1)  # 图像颠倒
                if ret == False or self.single_img is None:  # 如果没有图像
                    self.start_camera()  # 重新启动摄像头
                    self.take_photo_2()  # 重新进入这个函数
                    return
                else:
                    self.set_photo_flag(0)  # 设置为0，进入等待状态
                    self.event.set()  # 设置事件，保证拍照完成才进行推理

            elif self.isTake == 2:  # 停止拍摄状态
                self.release()  # 释放
                break
            cv2.waitKey(10)  # 进行一定的等待， 不用一只循环太多
        self.set_photo_flag(0)  # 设置拍照停止状态

    def take_photo_3(self):
        """
        3号拍摄方法 为了巡线的拍摄方法
        """

        while not rospy.is_shutdown():  # 节点没关闭
            ret, pic = self.cap.read()  # 读取图像
            self.single_img = cv2.flip(pic, 1)  # 图像颠倒
            if self.isTake == 1:  # 拍照状态
                if ret == False or self.single_img is None:  # 如果没有图像
                    self.start_camera()  # 重新启动摄像头
                    self.take_photo_3()  # 重新进入这个函数
                    return
                else:
                    self.set_photo_flag(0)  # 设置为0，进入等待状态
                    self.event.set()  # 设置事件，保证拍照完成才进行推理

            elif self.isTake == 2:  # 停止拍摄状态
                self.release()  # 释放
                break
            cv2.waitKey(10)  # 进行一定的等待， 不用一只循环太多
        self.set_photo_flag(0)  # 设置拍照停止状态
    
    def take_photo_4(self):
        """
            4号拍摄方法
            跟3一样,不过用在识别,先单独分出来
        """
        while not rospy.is_shutdown():  # 节点没关闭
            try:
                ret, pic = self.cap.read()  # 读取图像
                self.single_img = cv2.flip(pic, 1)  # 图像颠倒
            except Exception as E:
                print("something wrong to read the image!!!")
                self.start_camera()  # 重新启动摄像头
                self.take_photo_4()  # 重新进入这个函数
                return

            if self.isTake == 1:  # 拍照状态
                self.set_photo_flag(0)  # 设置为0，进入等待状态
                self.event.set()  # 设置事件，保证拍照完成才进行推理

            elif self.isTake == 2:  # 停止拍摄状态
                self.release()  # 释放
                break

            cv2.waitKey(10)  # 进行一定的等待， 不用一只循环太多
        self.set_photo_flag(0)  # 设置拍照停止状态

    def select_ref(self, mode):
        """选择推理方案"""
        result = None
        if mode == 0:  # 0号是测试
            result = self.reference_test()
        elif mode == 1:  # 1号单张图片保存检测
            result = self.reference_1()
        elif mode == 2:  # 2号单张图片线程间通信检测
            result = self.reference_2()
        else:
            pass
        return result

    def reference_test(self):
        """
        推理方案0 YoloV5的推理测试
        """
        self.set_photo_flag(1)  # 进入第一次设置为 1拍摄状态
        self.event.wait()  # 事件等待， 保证拍照完成
        results_bs = myPredict(model=self.model,
                               source=f"{folder_path}/data/my_imgs/frame2.jpg",
                               imgsz=self.imgsz, pt=self.pt,
                               stride=32)
        if len(results_bs) > 0:
            results = results_bs[0]  # result x1, y1, x2, y2, con, cls
            for result in results:
                self.save_test_img(result)  # 将结果保存, 保存路径是固定
        else:
            results = None

        self.event.clear()  # 将事件设置为假， 需要再次触发
        return results

    def reference_1(self):
        """
        推理方案1 从路径当中获取图片的方案
        """
        results = None
        return results
        pass #这个其实还没编写，有必要再编写

    def reference_2(self):
        """
        推理方案2 直接通过线程通信获取图片
        """
        self.set_photo_flag(1)  # 进入拍摄状态
        self.event.wait()  # 事件等待，保证拍摄完成
        
        
        results_bs = rknnPredict_single(im0=self.single_img, imgsz=self.imgsz, pt=self.pt,
                                      stride=32, rknn=self.rknn)

        if len(results_bs) > 0:  # 这部分也是暂时测试代码
            # results = results_bs[0]  # result x1, y1, x2, y2, con, cls #外层套了一个列表[]，因此要取0
            # 0708
            '''
            rospy.set_param("result_cam_0", results_bs[0, 0].item())
            rospy.set_param("result_cam_1", results_bs[0, 1].item())
            rospy.set_param("result_cam_2", results_bs[0, 2].item())
            rospy.set_param("result_cam_3", results_bs[0, 3].item())
            '''
        else:
            results = None
        # print(f"results:{results}")
        self.event.clear()  # 将事件设置为假， 需要再次触发
        return results_bs  # 返回推理结果

    def save_test_img(self, result):
        print("save")
        if self.saveIndex <= 1000:  # 最多不超过一千张，保护存储容量
            x1, y1, x2, y2 = result[:4]
            text = str(self.classes[int(result[5])]) + str(result[4])
            frame = cv2.imread(
                f"{folder_path}/data/my_imgs/frame2.jpg")  # 读取该张图片
            drawtext(frame, (int(x1), int(y1)), (int(x2), int(y2)), text)  # 绘制框和文本
            cv2.imwrite(self.test_save_path + str(self.saveIndex) + ".jpg", frame)  # 写入
            self.saveIndex += 1

    def set_photo_flag(self, flag):
        self.isTake = flag

    def release(self):
        self.cap.release()
        print(">>>> 已释放摄像头资源")


if __name__ == '__main__':  # 这个是测试代码
    from my_utils.play_voice import voice
    with open(f'{folder_path}/../../scripts/my_params/rec_params.yml', 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)  # 获取yolo识别策略的参数

    with open(f'{folder_path}/../../scripts/my_params/classes.yml', 'r') as f:  # 类别参数
        class_config = yaml.load(f, Loader=yaml.FullLoader)

    d = Detect(config, class_list=class_config['classes'])  # 创建检测对象
    print("加载完成")

    #测试方法
    k = 2
    if k == 1: #测试1
        thread = d.get_pic(mode=1)  # 1模式， 每set_photo_flag(1)拍摄一张

        i = 0
        while True:
            i += 1
            print(i)
            if i > 5:
                d.set_photo_flag(2)
                thread.join()
                break
            cv2.waitKey(50)
            result = d.select_ref(mode=0)  # 0模式，测试模式
            print(result)
            voice(class_config['classes'][int(result[0][5])])
    elif k == 2: #测试2 rescue_object_test
        thread = d.get_pic(mode=2) #2模式，拍摄图片但不保存
        i = 0
        while True:
            i += 1
            print(f"-----loop_{i}-----")
            cv2.waitKey(20)
            result = d.select_ref(mode=2) #2模式检测
            print("**reference:**")
            print(result)
            result = d.result_filter.select_filter(result, "rescue", 2) #救援物品
            # print("**result_filter**")
            if result != None and result != 0:
                to_mid = result[2][0] / d.width - 0.5
                area_rate = result[1] / (d.width * d.height)
                print(f"class: {result[0]}")
                print(f"to_mid: {to_mid}, area_rate: {area_rate}")       
                if abs(to_mid) < config["to_mid_thre"] and area_rate > config["area_rate_thre"]:    
                    if isinstance(result, list):
                        voice(result[0])
 
    elif k == 3: #测试3 terrorist_number_test
        thread = d.get_pic(mode=2) #2模式，拍摄图片但不保存
        i = 0
        while True:
            i += 1
            print(f"-----loop_{i}-----")
            cv2.waitKey(20)
            result = d.select_ref(mode=2) #2模式检测
            print("reference:")
            print(result)
            result = d.result_filter.select_filter(result, "terrorist", 2) #terrorist
            print("result_filter")
            print(result)
            if result!=0:
                if result == 1:
                    voice(d.classes[5])
                elif result == 2:
                    voice(d.classes[4])
                elif result == 3:
                    voice(d.classes[3])

    elif k == 4:
        #"bulletproof_vest"
        #"spontoon"
        #"teargas"
        target_object = "bulletproof_vest"
        thread = d.get_pic(mode=2) #2模式，拍摄图片但不保存
        i = 0
        while True:
            i += 1
            print(f"-----loop_{i}-----")
            cv2.waitKey(20)
            result = d.select_ref(mode=2) #2模式检测
            # print("**reference:**")
            print(result)
            result = d.result_filter.select_filter(result, "rescue", 4, target_object) #救援物品
            # print("**result_filter**")
            if result != None and result != 0:
                to_mid = result[2][0] / d.width - 0.5
                area_rate = result[1] / (d.width * d.height)
                print(f"class: {result[0]}")
                print(f"to_mid: {to_mid}, area_rate: {area_rate}")     
                if abs(to_mid) < config["to_mid_thre"] and area_rate > config["area_rate_thre"]:      
                    if isinstance(result, list):
                        voice(result[0])


            




