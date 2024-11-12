#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主预测流程函数
版本5微调
Luoxin
"""
import yaml
import numpy as np
import sys
import rospy
from time import sleep

import os

folder_path = os.path.dirname(os.path.abspath(__file__))
# ubuntu上用
sys.path.append(f"{folder_path}/../src/yolov5-master/")  # 添加yolo文件夹的路径
sys.path.append(f"{folder_path}/../../smartCatTTS/")  # 添加语音播报文件夹的路径

from my_detect import Detect
from my_trace import Trace

from my_utils.play_voice import voice  # 语音播报模块


class Rec:  # 我的识别类
    def __init__(self, class_config):
        """
        我的识别类
        """
        self.classes = class_config  # 获取类别数据

    def main(self, config, detect):
        """
        先到恐怖分子房间识别恐怖分子
        再经过坡道回来
        然后进入物品区域取物品
        最后从物品区域出去到SLAM区域或者巡线区域
        """
        rospy.init_node('my_rec')  # 启动识别节点
        terrorist_num = 0  # 识别到的恐怖分子的数量
        target_rescue = None  # 救援目标物品（在识别到恐怖分子后会匹配）
        thread = None  # 多开的线程

        is_finish_terrorist = False  # 是否探索完恐怖分子房间
        is_start_terrorist = False  # 是否开始探索恐怖分子房间

        print("----Rec参数----")
        for key, value in config.items():
            print(f"{key}: {value}")

        while not is_finish_terrorist:  # 首先探索恐怖分子区域，首先还没有探索完成
            if not is_start_terrorist:  # 是否是首次进入恐怖分子区域
                print("进入恐怖分子的房间")
                is_start_terrorist = True
                thread = detect.get_pic(mode=2)  # 开启2模式拍照线程(没有进入ref就会堵塞)
            else:  # 进入恐怖分子区域
                if rospy.get_param('take_photo', 0) == 1:  # 从参数服务器获取参数，默认为0，该参数由导航提供，现在是否拍照
                    result = detect.select_ref(mode=2)  # 2号模式预测得到预测结果(这个可以阻止拍照线程堵塞)
                    result = detect.result_filter.select_filter(result, "terrorist", 1)  # 结果过滤

                    if result != None and int(result) != 0:  # 如果结果不为0
                        rospy.set_param("terrorist", int(result))  # 给控制端反馈
                        voice(self.classes["classes"][6 - int(result)])  # 语音播报恐怖分子数量
                        print("ok_terrori")
                        terrorist_num = result  # 恐怖分子的数量
                        # 处理完成
                        rospy.set_param("take_photo", 0)  # 拍照完成, 重新设置为0
                        is_finish_terrorist = True

                        target_rescue = self.classes["terrorist_map"][terrorist_num]  # 需要找到的急救物品
                        break #摄像头此时还是保持开启的

        is_finish_first_aid_kit = False
        is_finish_rescue = False
        is_start_rescue = False

        print(f"targe rescue is {target_rescue}")
        while not is_finish_rescue:  # 再探测救援物品区域
            if not is_start_rescue:  # 进入救援物品区域
                print("进入救援物品的房间")
                is_start_rescue = True
            else:
                if rospy.get_param('first_aid_kit', 0) == 1: #急救包语音播报
                    print("have found first_aid_kit")
                    voice("first_aid_kit")
                    rospy.set_param('first_aid_kit', 0)
                    is_finish_first_aid_kit = True

                if rospy.get_param('take_photo', 0) == 1:  # 导航允许我们拍摄
                    result = detect.select_ref(mode=2)  # 经过神经网络，也是2号方
                    result = detect.result_filter.select_filter(result, "rescue", 2)  # 其他物品
                    # result = detect.result_filter.select_filter(result, "rescue", 4, target_rescue)  # 其他物品
                    print(result)
                    if result != None and result != 0:  # 如果有探测到物品，不管是什么
                        if result[0] == target_rescue:  # 如果结果为目标物品
                            to_mid = result[2][0] / detect.width - 0.5  # 目标偏离中心点的像素距离-0.5 - 0.5 从左到右
                            area_rate = result[1] / (detect.width * detect.height)  # 目标框的面积与图片面积比例 0 - 1
                            rospy.set_param("rescue_x", to_mid) #发送中点距离
                            rospy.set_param("rescue_area", area_rate) #发送区域面积
                            print(f"to_mid = {to_mid}, area_rate = {area_rate}")
                            if abs(to_mid)< 0.2:
                                rospy.set_param("rescue", 3) #找到了目标物品，但是不符合面积要求，导航可以通过这个和面积以及偏离度进行跟踪
                            if abs(to_mid) < config["to_mid_thre"] and area_rate > config["area_rate_thre"]:  #面积和中点偏离容忍度
                                rospy.set_param("rescue", 4)  # 完成寻找
                                rospy.set_param("take_photo", 0)  # 拍照完成
                                voice(result[0])  # 语音播报
                                detect.set_photo_flag(2)  # 关闭当前摄像头线程
                                thread.join()  # 等待线程执行完毕， 避免干扰后面
                                break  # 退出循环
                        else:
                            rospy.set_param("rescue", 1)  #找到了物品，但不是目标物品
                    else:
                        rospy.set_param("rescue", 0)

                elif rospy.get_param('take_photo', 0) == 2: #超时退出 take_photo要设置为2
                    rospy.set_param('take_photo', 0)
                    print("识别超时, 退出, 进行下下一个任务")
                    detect.set_photo_flag(2)  # 关闭当前摄像头线程
                    thread.join()  # 等待线程执行完毕， 避免干扰后面
                    break

        is_finish_trace = False
        is_start_trace = False

        if config["trace_line"] == 1:  # 是否进行巡线
            print("We will enter the trace room")
            is_start_trace = True
            while not is_finish_trace:  # 没有完成寻迹就一直循环
                if rospy.get_param("start_trace", 0) == 1: # 导航那边是否姿态调整完毕准备巡线了
                    trace = Trace(detect)  # 巡线类
                    trace.main()  # 巡线得
                    is_finish_trace = True  # 巡线完毕
                    break  # 跳出循环
                sleep(5)

        print(">>>> 等待停车指令")  # 最终停车的语音播报
        while True:
            sleep(1)
            if rospy.get_param('stop_flag', 0) == 1:  # 导航置入停车参数，说明小车已经到了停车位置
                voice("over")  # 结束语音播报
                rospy.set_param('stop_flag', 0)  # 停车标志复位，方便下次使用（因为rosmaster不停止）
                break


if __name__ == "__main__":  # 下面是真正运行的代码
    with open(f'{folder_path}/my_params/rec_params.yml', 'r') as f:  # 识别参数
        config = yaml.load(f, Loader=yaml.FullLoader)  # 全部读取为字典
    with open(f'{folder_path}/my_params/classes.yml', 'r') as f:  # 类别参数
        class_config = yaml.load(f, Loader=yaml.FullLoader)

    print("Yolo探测器启动")
    detect = Detect(config, class_list=class_config['classes'])  # 获取yolo探测器, 输入摄像头参数和类别列表信息
    my_rec = Rec(class_config)

    while not rospy.is_shutdown():  # 循环等待启动识别
        if rospy.get_param('rec_state', 0) == 1:  # 识别启动
            print("识别主程序main启动")
            my_rec.main(config, detect)  # 进入识别主程序
            rospy.set_param('rec_state', 0)  # 重新设置为0，相当于给了一个反馈
            break  # 退出循环
        rospy.sleep(5)
    print("over")