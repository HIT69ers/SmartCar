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
import pid_new
from judge_conditions_copy import *


def odom_callback(msg):
        print(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2])
        i = 0
        rospy.loginfo(f"{i}")
        

if __name__ == "__main__":
    rospy.init_node("test")
    boardnav = BoardNav()
    rospy.loginfo('>>>shot from laser info')
    position = rospy.wait_for_message('/cxy_base_link', Pose, timeout = 5)
    boardnav.now_position = (position.position.x, position.position.y)
    data = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout = 5)
    rospy.loginfo((position.position.x, position.position.y))
    boardnav.resolution = data.info.resolution
    boardnav.img = gen_cutPic(data.data)


    
