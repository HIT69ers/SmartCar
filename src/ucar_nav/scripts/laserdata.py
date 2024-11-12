#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import tf.transformations
import rospy
from std_msgs.msg import * 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from collections import deque
import numpy as np
from Utils import *
def callback(data):
    print(np.rad2deg(euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]))[2])

# 话题监听函数，其实主要是一个初始化过程
# 初始化节点，'laser_listener' 挺重要的，是节点（node）名称，一个py文件只能有一个。
# 代表开启一个进程！匿名参数也默认是这个
# 订阅的语句。先是你需要订阅的话题要对，然后是数据类型，然后是回调函数名字
# 最后的队列很重要，不管发布还是订阅，都有queue_size参数；
# 如果默认的话，发布的频率远高于你处理图片的速度，因此根本无法实时的显示
# 所以需要换为1，只接收最新的一个消息，其他的都丢了不管~
def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)
    # rospy.Subscriber("/scan", LaserScan,callback,queue_size = 1)
    rospy.Subscriber('/cxy_base_link', Pose,callback)
    rospy.spin()

if __name__ == '__main__':
    laser_listener()