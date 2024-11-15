#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    This is used for progress check in 17th, Nov.
    Recognize all objects.
    Sun Yuhang
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

# class for Recognition tasks
class Rec: 
    def __init__(self, class_config):
        self.classes = class_config  # classes data
    
    def main(self, config, detect):
        """
        1.Recognize terrorist
        2.Get first_aid_kit
        3.Get weapon
        """
        rospy.init_node('my_rec')
        terrorist_num = 0
        # target_rescue = None  # Target object(not used)
        thread = None
        cnt = 0

        is_finish_terrorist = False
        is_start_terrorist = False

        print('----Rec Params----')
        for key, value in config.items():
            print(f'{key}: {value}')
        rospy.set_param('flag_findornot_nav', 4)
        while not is_finish_terrorist:
            room = rospy.get_param('room', 0) # Room judgement 1.terrorist 2.first_aid_object
            if room == 1 or is_start_terrorist:
                if not is_start_terrorist:  # 是否是首次进入恐怖分子区域
                    print("进入恐怖分子的房间")
                    is_start_terrorist = True
                    thread = detect.get_pic(mode=2)
                
                if rospy.get_param('take_photo', 0) == 1:
                    result = detect.select_ref(mode=2)
                    result = detect.result_filter.select_filter(result, 'terrorist', 1)

                    if result is not None and int(result) != 0:
                        rospy.set_param('terrorist', int(result))  # Give feedback to control set
                        voice(self.classes['classes'][6 - int(result)])
                        print('ok_terrorist')
                        terrorist_num = result

                        rospy.set_param('take_photo', 0)
                        is_finish_terrorist = True

                        # target_rescue = self.classes['terrorist_map'][terrorist_num]
                        break

        is_finish_first_aid_kit = False
        is_finish_rescue = False
        is_start_rescue = False
        is_finish_finding = {
            'spontoon': False,
            'bulletproof_vest': False,
            'teargas': False
        }
        # area_rate_thre = config['area_rate'][target_rescue]
        
        while not is_finish_rescue:
            room = rospy.get_param('room', 0)
            room = 2
            if room == 2 or is_start_rescue:  # 进入救援物品区域
                if not is_start_rescue:  # 首次进入救援物品房间
                    print("进入救援物品的房间")
                    is_start_rescue = True
                
                if rospy.get_param('first_aid_kit', 0) == 1:
                    print("have found first_aid_kit")
                    voice("first_aid_kit")
                    rospy.set_param('finish_aid', 1)
                    rospy.set_param('first_aid_kit', 0)
                    is_finish_first_aid_kit = True
                else:
                    if rospy.get_param('take_photo', 0) == 1:
                        result = detect.select_ref(mode=2)
                        result = detect.result_filter.select_filter(result, 'rescue', 1)
                        if result:
                            rospy.set_param(result, 1)
                            rospy.set_param('take_photo', 0)
          
                if rospy.get_param('take_photo', 0) == 1:
                    result = detect.select_ref(mode=2)
                    result = detect.result_filter.select_filter(result, 'rescue', 2)
                    print(result)

                    if(result) != None and result != 0:
                        print(f"'flag_findornot_nav'是{rospy.get_param('flag_findornot_nav',  0)}")
                        print(f"'set_goal'是{rospy.get_param('set_goal',  0)}")
                        to_mid = result[2][0] / detect.width - 0.5
                        area_rate = result[1] / (detect.width * detect.height)
                        area_rate_thre = config['area_rate'][result[0]]
                        rospy.set_param('rescue_x', to_mid)
                        rospy.set_param('rescue_area', area_rate)
                        print(f'to_mid = {to_mid}, area_rate = {area_rate}')
                        if abs(to_mid) > 0.4:
                            rospy.set_param('rescue', 3)
                            print(f'rescue===={rospy.get_param("rescue", 0)}')
                        if abs(to_mid) < config['to_mid_thre'] and area_rate > area_rate_thre:
                            voice(result[0])
                            is_finish_finding[result[0]] = True
                            rospy.set_param('take_photo', 0)
                            if(is_finish_finding['spontoon'] and is_finish_finding['bulletproof_vest'] and is_finish_finding['teargas']):
                                rospy.set_param('rescue', 4)
                                detect.set_photo_flag(2)
                                thread.join()
                                break
                            else:
                                rospy.set_param('rescue', 5)
                    else:
                        if rospy.get_param('rescue', 0) != 5:
                            rospy.set_param('rescue', 0)
                
                elif rospy.get_param('take_photo', 0) == 2:
                    rospy.set_param('take_photo', 0)
                    print("识别超时, 退出, 进行下一个任务")
                    detect.set_photo_flag(2)  # 关闭当前摄像头线程
                    thread.join()  # 等待线程执行完毕， 避免干扰后面
                    break
        
        while not is_finish_first_aid_kit:
            if rospy.get_param('first_aid_kit', 0) == 1:
                print("have found first_aid_kit")
                voice("first_aid_kit")
                rospy.set_param('first_aid_kit', 0)
                is_finish_first_aid_kit = True
                break
        
        print('>>>> 等待停车指令')
        while True:
            sleep(1)
            if rospy.get_param('stop_flag', 0) == 1:
                voice('over')
                rospy.set_param('stop_flag', 0)
                break


if __name__ == '__main__':
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
