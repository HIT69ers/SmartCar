#!/usr/bin/env python 
# -*- coding: utf-8 -*-

from math import sqrt
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
import numpy as np
from tf.transformations import quaternion_from_euler


# 宏变量
a_bridge_first = 'xa_bridge_first'
a_bridge_last = 'xa_bridge_last'
f_bridge_first = 'mf_bridge_first'
f_bridge_last = 'mf_bridge_last'
start_p='start_p'
start_t='start_t'
# a_bridge_second = 'mf_bridge_second'
# f_bridge_second = 'za_bridge_second'

def nearby_1d(position, line, theta):
    """
    @param position: 小车所处的x或y坐标
    @param line: 希望达到的线
    @param theta: 间距
    @return position是否在line的theta邻域内
    """
    return line - theta < position < line + theta

def nearby_2d(position, point, theta):
    """ 
    @param position: 小车所处的坐标
    @param point: 希望达到的点
    @param theta: 间距
    @return positon 是/否 在point的theta邻域内
    """
    # x = position.pose.pose.position.x
    # y = position.pose.pose.position.y
    x, y = position
    px,py = point
    range = sqrt((px - x)**2 + (py - y)**2)
    return  range < theta

def find_next_key(order_dict, now_key):
    flag = 0
    for key in order_dict.keys():
        if flag == 1:
            return key
        if key == now_key:
            flag = 1
    return 'can find the next of your key'
        


def Twist_Pub(pub, lx, ly, az):
    """
    速度发布函数
    pub: pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    """
    twist = Twist()
    twist.linear.x = lx
    twist.linear.y = ly
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = az
    pub.publish(twist)

class Board():
    def __init__(self) -> None:
        self.position = None
        self.ang_pic2car = None
        self.aim_pose = None
        self.dir = None # 板子朝向的方向向量，单位向量
        self.uni_shot = False

    def __eq__(self, __value: object) -> bool:
        return np.dot(self.position - __value.position,(self.position - __value.position).T) < 1e-4
    #将板子的位置和朝向转化成为目标点
    def gen_pose(self, point , dir):
        pose = Pose() 
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = 0.0
        angle = np.arctan2(dir[1],dir[0])
        print(f'小车的期望位置为{point}')
        euler_angle = np.array([0,0,angle])
        print(f'小车期望朝向角度为{np.rad2deg(angle)}')
        print()
        q = quaternion_from_euler(*euler_angle)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self.aim_pose = pose

def get_point(board_position, dir, length):
    """
    返回length余量长度的点
    @param board_position: 障碍板中心点位置
    @param angle: 障碍板中心点与房间中心点连线与x轴夹角
    @param length: 余量长度
    """
    return board_position + length * dir

class ShotAngle():
    def __init__(self, angle, shot_type):
        self.angle = angle
        self.shot_type = shot_type

def sort_by_angle(obj):
    return obj.angle

def point_in_convex_polygon(point, polygon):
    # 将四边形的四个顶点坐标分别存储在四个变量中
    x1, y1, x2, y2, x3, y3, x4, y4 = polygon
    # 计算点与矩形四个顶点所形成的四个三角形的叉积
    cp1 = np.cross([x2 - x1, y2 - y1], [point[0] - x1, point[1] - y1])
    cp2 = np.cross([x3 - x2, y3 - y2], [point[0] - x2, point[1] - y2])
    cp3 = np.cross([x4 - x3, y4 - y3], [point[0] - x3, point[1] - y3])
    cp4 = np.cross([x1 - x4, y1 - y4], [point[0] - x4, point[1] - y4])
    # 如果四个叉积的符号都相同，则点在四边形内部
    if cp1 * cp2 >= 0 and cp2 * cp3 >= 0 and cp3 * cp4 >= 0:
        return True
    else:
        return False