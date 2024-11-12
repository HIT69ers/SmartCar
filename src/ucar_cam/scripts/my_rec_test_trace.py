#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主预测流程函数
"""
import yaml
import sys
import rospy

import os
folder_path = os.path.dirname(os.path.abspath(__file__))

# ubuntu上用
sys.path.append(f"{folder_path}/../src/yolov5-master/")  # 添加yolo文件夹的路径
sys.path.append(f"{folder_path}/../../smartCatTTS/")  # 添加语音播报文件夹的路径

from my_utils import point_in_convex_polygon
from my_detect import Detect
from my_trace import Trace

from geometry_msgs.msg import Pose  # 订阅的小车姿态的话题


class Rec:  # 我的识别类
    def __init__(self, class_config):
        """
        我的识别类
        """
        with open(f'{folder_path}/my_params/room.yml', 'r') as f:
            self.room = yaml.load(f, Loader=yaml.FullLoader)  # 获取room的坐标参数

        self.classes = class_config  # 获取类别数据

        self.position = Pose()
        self.position.position.x = -1.0
        self.position.position.y = -1.0

    def judge_room(self, position_x, position_y):
        """
        判断小车坐标是否在某个房间中
        """
        for room_name, area_p in self.room.items():
            if point_in_convex_polygon((position_x, position_y), area_p):  # 判断点是否在房间（四边形）中
                return room_name  # 返回房间号
        return None  # 如果里面没有的话就返回None

    def room_pos_callback(self):  # 返回订阅对象的回调函数
        """闭包 返回设置位置的函数"""

        def callback(msg):  # 一个回调函数，通过话题获取当前位置信息
            """通过/cxy_base_link话题获取当前位置信息"""
            self.position = msg

        return callback

    def which_room(self):
        """判断房间的集成"""
        room = self.judge_room(self.position.position.x, self.position.position.y)
        return room

    def main(self, config, detect):
        """测试寻迹"""
        rospy.init_node('my_rec')  # 启动识别节点
        print("/cxy_base_link话题订阅对象建立")
        pos_sub = rospy.Subscriber('/cxy_base_link', Pose, self.room_pos_callback())  # 位置话题订阅器

        is_finish_trace = False
        is_start_trace = False

        if rospy.get_param('trace_line', 0):  # 是否进行巡线，导航部分这个参数应该在前面设置好了
            print("需要寻迹")
            while not is_finish_trace:  # 没有完成寻迹就一直循环
                room = self.which_room()  # 房间判断
                print(room)
                if room == "trace_room" or is_start_trace:  # 进入巡线房间
                    if not is_start_trace:  # 是否首次进入巡线房间
                        print("We enter the trace room")
                        is_start_trace = True

                    if rospy.get_param("start_trace", 0) == 1:  # 导航那边是否姿态调整完毕准备巡线了
                        print("开始巡线")
                        trace = Trace(detect)  # 巡线类
                        trace.main()  # 巡线得
                        is_finish_trace = True  # 巡线完毕
                        print("巡线完毕")
                        break  # 跳出循环

        print("over")


if __name__ == "__main__":  # 下面是真正运行的代码
    with open(f'{folder_path}/my_params/rec_params.yml', 'r') as f:  # 识别参数
        config = yaml.load(f, Loader=yaml.FullLoader)  # 全部读取为字典
    with open(f'{folder_path}/my_params/classes.yml', 'r') as f:  # 类别参数
        class_config = yaml.load(f, Loader=yaml.FullLoader)

    print("Yolo探测器启动")
    detect = Detect(config, class_list=class_config['classes'])  # 获取yolo探测器, 输入摄像头参数和类别列表信息
    my_rec = Rec(class_config)

    print("节点循环")
    while not rospy.is_shutdown():  # 循环等待启动识别
        rospy.sleep(0.2)
        if rospy.get_param('rec_state', 0) == 1:  # 识别启动
                print("识别主程序main启动")
                my_rec.main(config, detect)  # 进入识别主程序
                rospy.set_param('rec_state', 0)  # 重新设置为0，相当于给了一个反馈
                break #退出循环

    print("over")
