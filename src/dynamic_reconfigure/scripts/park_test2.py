#!/usr/bin/env python
# -*- coding: utf-8 -*-

#####################################################

# Author : 路鑫
# Date: 2023.04.26

#####################################################

import sys
import rospy
import roslib
import dynamic_reconfigure.client
import math
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32


def Callback(config):
    inflation_radius = config.inflation_radius
    rospy.loginfo("Successfully")

if __name__ == "__main__":
    rospy.init_node("dynamic_client_local")
    rospy.set_param("/mission/mic_awake", False)
    rospy.set_param("/mission/park_ready", False)
    parking = False
    rate = rospy.Rate(5)
    # rospy.logwarn("______dynamic_readying__________________")
    while not rospy.is_shutdown():
        mic_awake = rospy.get_param("/mission/mic_awake")
        if mic_awake:
            break
        rate.sleep()
    client = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer", timeout = 30, config_callback = Callback)
    while  not rospy.is_shutdown():
        # rospy.logwarn("______parking_readying__________________")
        parking  = rospy.get_param("/mission/park_ready")
        var = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout = None)
        x = var.pose.pose.position.x
        y = var.pose.pose.position.y
        if x > 0 and x < 2.8 and y > -1.5 and y < 0.15 and parking :
            #A parking
            # rospy.logwarn("________________进入a房间拉__________________")
            client.update_configuration({"inflation_radius": 0.01})    #    0.05
            client.update_configuration({"cost_scaling_factor": 200.0})    #    0.05
            # client2.update_configuration({"inflation_radius": 0.03})    #    0.05
            # client.update_configuration({"weight_inflation": 100})
            # client.update_configuration({"weight_acc_lim_x": 1})
            # client.update_configuration({"weight_acc_lim_x": 1})
            # client.update_configuration({"weight_acc_lim_theta": 2})
            # client.update_configuration({"acc_lim_x": 0.4})
            # client.update_configuration({"     acc_lim_y": 0.4})
            # client.update_configuration({"acc_lim_theta": 2})
            # client.update_configuration({"max_vel_x": 1})
            # client.update_configuration({"max_vel_y": 1})
            # client.update_configuration({"xy_goal_tolerance": 0.03})
            # client.update_configuration({"xy_goal_tolerance": 0.01})
            # client.update_configuration({"yaw_goal_tolerance": 0.2})
            rospy.logwarn("____________局部膨胀半径设置完啦__________________")

        else :
            pass
