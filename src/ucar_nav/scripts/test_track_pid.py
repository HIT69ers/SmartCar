#!/usr/bin/env python 
# -*- coding: utf-8 -*-
"""
    使用pid控制小车沿着直线行走以及停车
    y给正值向左走
"""
import rospy
from std_msgs.msg import * 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from collections import deque
import numpy as np
from Utils import *
def test_trace_callback(msg):
    if rospy.get_param('nongoal_flag') == 2 :
        rospy.set_param('trace_speed', 1)
        if rospy.get_param('trace_speed', 0) == 0:
            xspeed = 0.6
            zspeed = pid_track.pidCalT(msg.data-94.0) # 给偏差
        else:
            xspeed = 0.5
            zspeed = pid_track.pidCalT(msg.data-94.0) # 给偏差
        Twist_Pub(pub_cmd, xspeed, 0, zspeed)
        rospy.loginfo(msg)
        rospy.loginfo(xspeed)
        rospy.loginfo(zspeed)




class pidControlerTRotate:
    # 转向PID
    def __init__(self, k):
        # 目标角度
        self.k = k
        # # speed:0.3#bias = -3
        # self.kp = 0.0165
        # self.ki = 0.0
        # self.kd = 0.08
        # # speed:0.4#bias = 1
        # self.kp = 0.017
        # self.ki = 0.00
        # self.kd = 0.08
        # # speed:0.5#bias = 2
        self.kp = 0.017
        self.ki = 0.00
        self.kd = 0.115
        # speed:0.6#bias = 6
        # self.kp = 0.018
        # self.ki = 0.00
        # self.kd = 0.125
        # self.kp = 0.017
        # self.ki = 0.00
        # self.kd = 0.125
        # 积分分离
        self.sumDiv = 20
        # pid 最终限制幅度
        self.pidLimit = 5

        self.lastError = 0
        self.errors = deque(maxlen=10)

    # 控制转向
    def pidCalT(self, w):
        error = self.k - w
        if error <= self.sumDiv:
            self.errors.append(error)
        if rospy.get_param('trace_speed', 0) == 0:
            p = 0.0165 * error
            i = 0 * sum(self.errors)
            d = 0.08 * (error - self.lastError)
        else:
            p = self.kp * error
            i = self.ki * sum(self.errors)
            d = self.kd * (error - self.lastError)
        self.lastError = error
        return min(self.pidLimit, p + i + d) 
class pidControlerXSpeed:
    def __init__(self, x_desire,kp,ki,kd):
                                                                     
        self.kp = kp        #10
        self.ki = ki            #0
        self.kd = kd      #0.5

        # 积分分离
        self.sumDiv = 20

        # pid 最终限制幅度
        self.pidLimit = 1.0
        self.lastError = 0
        self.errors = deque(maxlen=10)

    # 控制水平速度
    def pidCalX(self, x_now):
        error =  self.line_x - x_now
        if error <= self.sumDiv:
            self.errors.append(error)
        p = self.kp * error
        i = self.ki * sum(self.errors)
        d = self.kd * (error - self.lastError)
        self.lastError = error
        if p + i + d>self.pidLimit:
            return self.pidLimit
        elif p + i + d<-self.pidLimit:
            return -self.pidLimit
        else:
            return p + i + d
        
class pidControlerYSpeed:
    def __init__(self, y_desire,kp,ki,kd):
        self.line_y = y_desire

        self.kp = kp
        self.ki = ki
        self.kd = kd

        # 积分分离
        self.sumDiv = 20

        # pid 最终限制幅度
        self.pidLimit = 1.0
        self.lastError = 0
        self.errors = deque(maxlen=10)

    # 控制水平速度
    def pidCalY(self, y_now):
        error =  self.line_y - y_now
        if error <= self.sumDiv:
            self.errors.append(error)
        p = self.kp * error
        i = self.ki * sum(self.errors)
        d = self.kd * (error - self.lastError)
        self.lastError = error
        if p + i + d>self.pidLimit:
            return self.pidLimit
        elif p + i + d<-self.pidLimit:
            return -self.pidLimit
        else:
            return p + i + d
    
class pidControlerWRotate:
    def __init__(self, k):
        # 目标角度
        self.k = k


        self.kp = 1
        self.ki = 0
        self.kd = 0
        # self.kp = 5
        # self.ki = 0.36
        # self.kd = 0.1

        # 积分分离
        self.sumDiv = 20
        # pid 最终限制幅度
        self.pidLimit = 5

        self.lastError = 0
        self.errors = deque(maxlen=10)

    # 控制角度
    def pidCalW(self, w):
        error = self.k - w
        if error <= self.sumDiv:
            self.errors.append(error)
        p = self.kp * error
        i = self.ki * sum(self.errors)
        d = self.kd * (error - self.lastError)
        self.lastError = error
        if p + i + d>self.pidLimit:
            return self.pidLimit
        elif p + i + d<-self.pidLimit:
            return -self.pidLimit
        else:
            return p + i + d
        



if __name__ == "__main__":
    rospy.init_node("nongoal")
    rospy.set_param('orientation', 0)  #
    rospy.set_param('nongoal_flag', 0)  # 0表示停止，1表示A->S, 2表示F->A
    rospy.loginfo("pid ready")
    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    track_sub = rospy.Subscriber('/mid_topic', Float32,test_trace_callback)

    pid_track = pidControlerTRotate(0) # 初始化-期望值给偏差为0
    
    rospy.spin()
    