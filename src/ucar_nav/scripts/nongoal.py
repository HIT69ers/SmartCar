#!/usr/bin/env python 
# -*- coding: utf-8 -*-
"""
    使用pid控制小车沿着直线行走以及停车
"""
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from collections import deque
import numpy as np
from Utils import Twist_Pub

def odom_callback(msg):
    if rospy.get_param('nongoal_flag') == 1 and rospy.get_param('stop_mode', 0) == 0:
        if msg.position.y > -0.225 and msg.position.x < 0.25:
            rospy.set_param('nongoal_flag', 0)
            rospy.set_param("if_stop_flag",1)
            end_time = rospy.Time.now()
            rospy.loginfo(">>>用时" + str(end_time.to_sec() - rospy.get_param('start_time')) + 's')
        xspeed = park_pidx.pidCalX(np.array([msg.position.y])) if park_pidx.goal_point > msg.position.y else 0
        yspeed = park_pidy.pidCalX(np.array([msg.position.x])) if park_pidy.goal_point < msg.position.x else 0
        zspeed = pidw.pidCalW(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2])
        Twist_Pub(pub_cmd, xspeed, yspeed, zspeed)
    elif rospy.get_param('nongoal_flag') == 2:
        if msg.position.y > - 0.8:  # y坐标大于这个值时，开始pid停车过程
            rospy.set_param('nongoal_flag', 1)  # 控制转向pid停车
        yspeed = pidy.pidCalY((msg.position.x, msg.position.y))
        zspeed = pidw.pidCalW(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2])
        Twist_Pub(pub_cmd, 1, yspeed, zspeed)
    elif rospy.get_param('nongoal_flag') == 3:
        if msg.position.y < - 2.20:  # y坐标小于这个值时，切换到多点导航，同时需要给全局地图留出更新时间 
            rospy.set_param('nongoal_flag', 0)
        yspeed = pidy2.pidCalY((msg.position.x, msg.position.y))
        zspeed = pidw2.pidCalW(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2])
        Twist_Pub(pub_cmd, 1, - yspeed, zspeed)    
        

class pidControlerWRotate:
    def __init__(self, k):
        # 目标角度
        self.k = k

        self.kp = 4
        self.ki = 0.03
        self.kd = 0.1
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
        return min(self.pidLimit, p + i + d)
    
def car_track(bias_track):
    xspeed = 1
    yspeed = 0
    zspeed = pid_track.pidCalT(bias_track) # 应该输入图像返回的偏差
    Twist_Pub(pub_cmd, xspeed, yspeed, zspeed) 
    
class pidControlerTRotate:
    # 转向PID
    def __init__(self, k):
        # 目标角度
        self.k = k
        self.kp = 4
        self.ki = 0.03
        self.kd = 0.1

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
        p = self.kp * error
        i = self.ki * sum(self.errors)
        d = self.kd * (error - self.lastError)
        self.lastError = error
        return min(self.pidLimit, p + i + d) 

class pidControlerWRotate:
    def __init__(self, k):
        # 目标角度
        self.k = k

        self.kp = 4
        self.ki = 0.03
        self.kd = 0.1
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
        return min(self.pidLimit, p + i + d)

class pidControlerXSpeed:
    def __init__(self, goal_point, kp):
        self.kp = kp
        self.ki = 0.15
        self.kd = 0
        self.goal_point = goal_point

        # 积分分离
        self.sumDiv = 20

        # pid 最终限制幅度
        self.pidLimit = 1.5
        self.lastError = 0
        self.errors = deque(maxlen=10)

    def pidCalX(self, now_point):
        """根据当前位置计算x方向的速度"""
        error = np.sqrt(np.dot(now_point-self.goal_point, (now_point-self.goal_point).T))
        p = self.kp * error
        if error < self.sumDiv:
            i = self.ki * sum(self.errors)
            self.errors.append(error)
        else:
            i = 0
        d = self.kd * (error - self.lastError)
        self.lastError = error
        return min(self.pidLimit, p + i + d)
        
class pidControlerYSpeed:
    def __init__(self, line_point1, line_point2):
        self.line_point1 = np.array(line_point1)
        self.line_point2 = np.array(line_point2)

        self.kp = 0.9
        self.ki = 0.1
        self.kd = 0.0

        # 积分分离
        self.sumDiv = 20

        # pid 最终限制幅度
        self.pidLimit = 1.0
        self.lastError = 0
        self.errors = deque(maxlen=10)

    def point_distance_line(self, point):
        # 计算向量
        vec1 = self.line_point1 - point
        vec2 = self.line_point2 - point
        distance = np.cross(vec1, vec2) / np.linalg.norm(self.line_point1 - self.line_point2)
        return distance

    # 控制水平速度
    def pidCalY(self, point):
        error = self.point_distance_line(point)
        if error <= self.sumDiv:
            self.errors.append(error)
        p = self.kp * error
        i = self.ki * sum(self.errors)
        d = self.kd * (error - self.lastError)
        self.lastError = error
        return - min(self.pidLimit, p + i + d)

if __name__ == "__main__":
    rospy.init_node("nongoal")
    rospy.set_param('orientation', 0)  #
    rospy.set_param('nongoal_flag', 0)  # 0表示停止，1表示A->S, 2表示F->A
    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    odom_sub = rospy.Subscriber('/cxy_base_link', Pose, odom_callback)
    F2Ap1 = (1.02, -2.356)
    F2Ap2 = (1.02, -0.308)
    # A2Fp1 = (0.95, -2.356)
    # A2Fp2 = (0.95, -0.308)
    A2Fp1 = (-0.0666,-0.0175)
    A2Fp2 = (-1.411,-0.105)

    pidy = pidControlerYSpeed(F2Ap1, F2Ap2) # F <-> A
    pidy2 = pidControlerYSpeed(A2Fp1, A2Fp2) # F <-> A
    pidw = pidControlerWRotate(np.arccos(np.dot((F2Ap2[0]-F2Ap1[0], F2Ap2[1]-F2Ap1[1]), [1, 0]) / np.linalg.norm((F2Ap2[0]-F2Ap1[0], F2Ap2[1]-F2Ap1[1])))) # F -> A
    pidw2 = pidControlerWRotate(- np.arccos(np.dot((A2Fp2[0]-A2Fp1[0], A2Fp2[1]-A2Fp1[1]), [1, 0]) / np.linalg.norm((A2Fp2[0]-A2Fp1[0], A2Fp2[1]-A2Fp1[1])))) # A -> F
    park_pidx = pidControlerXSpeed(np.array([-0.060]), 0.8) # y坐标
    park_pidy = pidControlerXSpeed(np.array([-0.008]), 0.8) # x坐标
    pid_track = pidControlerTRotate(0) # 初始化-期望值给偏差为0
    rospy.spin()
    
