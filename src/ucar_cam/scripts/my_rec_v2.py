#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主预测流程函数
版本2改动的是不需要进行方位的接收判断，而是由导航那边判断是在哪个房
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
            room = rospy.get_param("room", 0)  # 房间判断, 1是恐怖分子房间， 2是救援物品房间， 3是寻迹房间
            if room == 1 or is_start_terrorist:  # 进入恐怖分子区域
                if not is_start_terrorist:  # 是否是首次进入恐怖分子区域
                    print("进入恐怖分子的房间")
                    is_start_terrorist = True
                    thread = detect.get_pic(mode=2)  # 开启2模式拍照线程(没有进入ref就会堵塞)

                if rospy.get_param('take_photo', 0) == 1:  # 从参数服务器获取参数，默认为0，该参数由导航提供，现在是否拍照
                    result = detect.select_ref(mode=2)  # 2号模式预测得到预测结果(这个可以阻止拍照线程堵塞)
                    result = detect.result_filter.select_filter(result, "terrorist", 1)  # 结果过滤

                    if result != None and int(result) != 0:  # 如果结果不为0
                        rospy.set_param("terrorist", int(result))  # 给控制端反馈
                        voice(self.classes["classes"][6 - int(result)])  # 语音播报恐怖分子数量
                        terrorist_num = result  # 恐怖分子的数量
                        # 处理完成
                        rospy.set_param("take_photo", 0)  # 拍照完成, 重新设置为0
                        is_finish_terrorist = True

                        detect.set_photo_flag(2)  # 关闭当前拍摄线程， 因为恐怖分子区域和救援物品区域有一定距离，因此先关闭，节约中间的线程资源
                        thread.join()  # 等待该线程执行完毕，避免干扰后面
                        target_rescue = self.classes["terrorist_map"][terrorist_num]  # 需要找到的急救物品
                        break
        
        is_finish_first_aid_kit = False
        print('wait for first aid kit...')
        while not is_finish_first_aid_kit:
            if rospy.get_param('first_aid_kit', 0) == 1:
                print("have found first_aid_kit")
                voice("first_aid_kit")
                rospy.set_param('first_aid_kit', 0)    
                is_finish_first_aid_kit = True   
                break      

        is_finish_rescue = True
        is_start_rescue = False 

        found_first_aid_kit = False
        while not is_finish_rescue:  # 再探测救援物品区域
            room = rospy.get_param('room', 0)
            if room == 2 or is_start_rescue:  # 进入救援物品区域
                if not is_start_rescue:  # 首次进入救援物品房间
                    print("进入救援物品的房间")
                    is_start_rescue = True
                    thread = detect.get_pic(mode=2)  # 创建拍照线程

                if rospy.get_param('take_photo', 0) == 1:  # 导航允许我们拍摄
                    result = detect.select_ref(mode=2)  # 经过神经网络，也是2号方法

                    # 下面两个过滤输出的result结果是不相同的
                    if not found_first_aid_kit:  # 如果还没找到医疗包就找医疗包
                        result = detect.result_filter.select_filter(result, "rescue", 1)  # 急救包结果过滤
                    else:  # 如果找到了医疗包，就找其他救援物品
                        result = detect.result_filter.select_filter(result, "rescue", 2)  # 其他物品

                    if result == "first_aid_kit":  # 如果是急救包
                        rospy.set_param("first_aid_kit", result)  # 给控制端反馈, 完成急救包的探索
                        found_first_aid_kit = True
                        voice(result)  # 语音播报急救包
                    elif result != 0 and found_first_aid_kit:  # 如果之前已经找到急救包，而且此时预测结果不为0
                        if result == target_rescue:  # 如果结果为目标物品
                            to_mid = result[3][0] - detect.width / 2  # 目标偏离中心点的像素距离
                            area_rate = result[2] / (detect.width * detect.height)  # 目标框的面积与图片面积比例
                            rospy.set_param("rescue_x", to_mid)
                            rospy.set_param("rescue_area", area_rate)
                            if abs(to_mid) < config["to_mid_thre"] and area_rate < config["area_rate_thre"]:
                                rospy.set_param("rescue", 2)  # 完成寻找
                                voice(result[0])  # 语音播报
                                if rospy.get_param("is_finish_rescue", 0) == 1:  # 完成全部救援物品的探索
                                    is_finish_rescue = True  # 循环标志停止*-9+
                                rospy.set_param("take_photo", 0)  # 拍照完成,这句可以导航那边写，也可以我们这边写

                                detect.set_photo_flag(2)  # 关闭当前摄像头线程
                                thread.join()  # 等待线程执行完毕， 避免干扰后面
                                break  # 退出循环
                            else:
                                rospy.set_param("rescue", 1)
                                # 给导航部分发送信息说自己找到了需要的物品， 导航接收这个后再查看上面两个参数(时间顺序不要乱)

        is_finish_trace = False
        is_start_trace = False

        if rospy.get_param('trace_line', 0) == 1:  # 是否进行巡线，导航部分这个参数应该在前面设置好了
            while not is_finish_trace:  # 没有完成寻迹就一直循环
                room = rospy.get_param('room', 0)  # 房间判断
                if room == 3 or is_start_trace:  # 进入巡线房间
                    if not is_start_trace:  # 是否首次进入巡线房间
                        print("We enter the trace room")
                        is_start_trace = True

                    if rospy.get_param("start_trace", 0) == 1:  # 导航那边是否姿态调整完毕准备巡线了
                        trace = Trace(detect)  # 巡线类
                        trace.main()  # 巡线得
                        is_finish_trace = True  # 巡线完毕
                        break  # 跳出循环

        # 最终停车的语音播报
        while True:
            print(">>>> 等待停车指令")
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
            # except Exception as e:
            #     print("识别主程序main运行失败")
            #     rospy.set_param('rec_state', 2)  # 重新设置为2, 异常反馈
        rospy.sleep(5)
    print("over")

