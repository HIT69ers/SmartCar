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
def trace_callback(msg):
    if rospy.get_param('nongoal_flag') == 2 :
        if rospy.get_param('trace_speed', 0) == 0:
            xspeed = 0.3
            zspeed = pid_track.pidCalT(msg.data-94.0) # 给偏差
        else:
            xspeed = 0.45
            zspeed = pid_track.pidCalT(msg.data-94.0) # 给偏差
        Twist_Pub(pub_cmd, xspeed, 0, zspeed)
        rospy.loginfo(msg)
        rospy.loginfo(xspeed)
        rospy.loginfo(zspeed)


def odom_callback(msg):
    # if msg.position.y >0.1:
    #     Twist_Pub(pub_cmd, 0,  0,0 )    
    # 0515
    if msg.position.y >-0.3 and msg.position.x >1 and rospy.get_param('nongoal_flag') ==2:  # x坐标小于这个值时，切换到多点导航，同时需要给全局地图留出更新时间 
        rospy.set_param('nongoal_flag', 1)
        rospy.loginfo("pid_trace_end")
    if rospy.get_param('nongoal_flag') == 1 :
        if nearby_2d((msg.position.x,msg.position.y),end_point,0.6):   
            rospy.set_param('stop_flag', 1)
            rospy.set_param('stop_trace', 1) 
            rospy.set_param('nongoal_flag', 0)
            rospy.loginfo("parking end")
        yspeed = park_pidx.pidCalX(( msg.position.x))
        xspeed = park_pidy.pidCalY(( msg.position.y))
        Twist_Pub(pub_cmd, xspeed, -yspeed, 0)
        rospy.loginfo(xspeed)
        rospy.loginfo(yspeed)

    elif rospy.get_param('nongoal_flag') == 2:
        if msg.position.y >-0.7 and msg.position.x <1:
            rospy.set_param('trace_speed',0)
        else:
            rospy.set_param('trace_speed',1)
        # rospy.set_param('trace_speed',1)
    #     xspeed = 0.2
    #     zspeed = pid_track.pidCalT(msg) # 给偏差
    #     Twist_Pub(pub_cmd, 1, xspeed, zspeed)
    elif rospy.get_param('nongoal_flag') == 3:
        if msg.position.x < - 1.4:  # x坐标小于这个值时，切换到多点导航，同时需要给全局地图留出更新时间 
            rospy.set_param('nongoal_flag', 0)
            rospy.loginfo("pid_reach")
        yspeed = pidy1.pidCalY(( msg.position.y))
        zspeed = pidw1.pidCalW(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2])
        Twist_Pub(pub_cmd, 1.1,  -yspeed, 0) 
        # Twist_Pub(pub_cmd, 1,  -yspeed, 0) 

    elif rospy.get_param('nongoal_flag') == 4 :
        rospy.set_param('rotate_last_flag', 0) 
        if msg.position.x > -0.5:  # x坐标小于这个值时，切换到多点导航，同时需要给全局地图留出更新时间 -0.453
            rospy.set_param('nongoal_flag', 5)
        yspeed = pidy2.pidCalY(( msg.position.y))
        Twist_Pub(pub_cmd, 1.0,  yspeed, 0)
        # Twist_Pub(pub_cmd, 0.7,  yspeed, 0)
        rospy.loginfo(yspeed)
        

    elif rospy.get_param('nongoal_flag') == 5: #转90
        xspeed = trace_pidx.pidCalX(( msg.position.x))
        yspeed = trace_pidy.pidCalY(( msg.position.y))
        zspeed=0
        if msg.position.x>-0.35 and msg.position.y< 0.03:   
            rospy.set_param('rotate_last_flag', 1) 
        if rospy.get_param('rotate_last_flag') == 1:   
            xspeed=0
            yspeed=0
            zspeed = pid_track_start.pidCalW(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2])
            if abs(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2]+1.74)<0.16:#1.9198
                
                rospy.set_param('start_trace', 1) 
                rospy.set_param('nongoal_flag',2)
                rospy.loginfo("pid_reach")
                rospy.set_param('take_photo', 2)  
                 
        Twist_Pub(pub_cmd, xspeed,  yspeed,zspeed )
    

    elif rospy.get_param('nongoal_flag') == 6 : #找板子
        pid_findboardw.k = rospy.get_param('angel_target_boundary_rad', 0)
            
        if rospy.get_param('rescue')==4 or (msg.position.x<-2.6 or msg.position.x>-0.4 or msg.position.y>1.4 or msg.position.y<-1.9):  # x坐标小于这个值时，切换到多点导航，同时需要给全局地图留出更新时间 -0.453
            rospy.set_param('take_photo',  0) 
            xspeed = 0
            yspeed = 0
            rospy.sleep(2)
            rospy.set_param('nongoal_flag', 0)
            rospy.loginfo('end findboard')
        xspeed = pid_findboardx.pidCalX(rospy.get_param('rescue_area'))
        # xspeed = 0
        yspeed = pid_findboardy.pidCalY(rospy.get_param('rescue_x'))
        zspeed = 0
        if  rospy.get_param('rescue_x')<0.15:
            if(rospy.get_param('flag_findornot_nav', 0) == 0):#如果导航没有找到目标版
                # yspeed=0
                print(f"PID1")
                xspeed=pid_findboardx.pidCalX(rospy.get_param('rescue_area'))
                # zspeed = pid_findboardw.pidCalW(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2])
                # print(f"PID输入{euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2]}")
                # print(f"PID期望{pid_findboardw.k}")
                # print()
                # if abs(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2]-pid_findboardw.k)<0.16:#1.9198
                #     xspeed=pid_findboardx.pidCalX(rospy.get_param('rescue_area'))
            else:
                # print(f"PID2")
                # zspeed = 0
                xspeed=pid_findboardx.pidCalX(rospy.get_param('rescue_area'))
        Twist_Pub(pub_cmd, xspeed,  yspeed, zspeed)   
        rospy.loginfo(xspeed)
        rospy.loginfo(yspeed)
        
    elif rospy.get_param('nongoal_flag') == 7: #导航停车
        if nearby_2d((msg.position.x,msg.position.y),end_point2,0.03):   
            xspeed =0
            yspeed =0
            rospy.set_param('stop_flag', 1)
            rospy.set_param('ready_stop', 0)
            rospy.loginfo("parking end")
        xspeed = park_pidx2.pidCalX(( msg.position.x))
        yspeed = park_pidy2.pidCalY(( msg.position.y))
        Twist_Pub(pub_cmd, xspeed, yspeed, 0)


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
        # self.kp = 0.017
        # self.ki = 0.00
        # self.kd = 0.115
        # speed:0.6#bias = 5
        self.kp = 0.018
        self.ki = 0.00
        self.kd = 0.125
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
        self.line_x = x_desire

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
    odom_sub = rospy.Subscriber('/cxy_base_link', Pose, odom_callback)
    track_sub = rospy.Subscriber('/mid_topic', Float32,trace_callback)


    y_bridge_f=-0.07                                                            #第一次上桥点
    y_bridge_s=0.05            # before 0.03                                       #第二次上桥点
    
    A_point=(0.86, 1.33)                                                     #识别恐怖分子的点
    B_point=(-2.487, -1.83)                                               #急救包点

    # trace_start_point=(-0.33,-0.08)                                 #巡迹起始点坐标期望值
    # trace_start_point=(-0.25,0.03)  #0628
    trace_start_point=(-0.3,0.03)  
    A2Fpoint1=(-0.0666,-0.0175)
    A2Fpoint2=(-1.411,-0.105)

    # end_point=(1.83, -0.15)                                                      #巡线停车期望点
    end_point=(1.9, -0.20)
    # end_point=(1.9, -0.17)
    end_point2=(1.7, -0.05)     
    #新地图取点
    # y_bridge_s=0
    # trace_start_point=(-0.38,0)
    # end_point=(1.811, -0.54)  

    # pid_a_x=pidControlerXSpeed(A_point[0],5,0,0.1)
    # pid_a_y=pidControlerYSpeed(A_point[1],5,0,0.1) 

    # pid_b_x=pidControlerXSpeed(B_point[0],5,0,0.1)
    # pid_b_y=pidControlerYSpeed(B_point[1],5,0,0.1) 

    pidy1 = pidControlerYSpeed(y_bridge_f,10,0,0.5) # 第一次上桥
    pidy2= pidControlerYSpeed(y_bridge_s,5,0,0.1) # 第二次上桥
    pidw1 = pidControlerWRotate( np.arccos(np.dot((A2Fpoint2[0]-A2Fpoint1[0], A2Fpoint2[1]-A2Fpoint1[1]), [-1, 0]) / np.linalg.norm((A2Fpoint2[0]-A2Fpoint1[0], A2Fpoint2[1]-A2Fpoint1[1])))) # 第一次上桥角度

    trace_pidx=pidControlerXSpeed(trace_start_point[0],10,0,0.5)
    trace_pidy=pidControlerYSpeed(trace_start_point[1],10,0,0.5)   
    pid_track_start=pidControlerWRotate(-1.74)

    park_pidx=pidControlerXSpeed(end_point[0],8,0,0.5)                  #巡线停车
    park_pidy=pidControlerYSpeed(end_point[1],8,0,0.5)
    pid_track = pidControlerTRotate(0) # 初始化-期望值给偏差为0

    park_pidx2=pidControlerXSpeed(end_point2[0],1,0,0.1)            #导航停车
    park_pidy2=pidControlerYSpeed(end_point2[1],1,0,0.1)

    pid_findboardx=pidControlerXSpeed(0.75,0.5,0,0.1)
    pid_findboardy=pidControlerYSpeed(0,0.8,0,0.1)
    
    #0511
    pid_findboardw=pidControlerWRotate(rospy.get_param('angel_target_boundary_rad', 0))#已经转换成弧度制
    # pid_findboardy=pidControlerYSpeed(0,2,0,0.1)
    rospy.spin()
    