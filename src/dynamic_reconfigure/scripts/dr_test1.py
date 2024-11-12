#!/usr/bin/env python
# -*- coding: utf-8 -*-

#####################################################

# Author : 路鑫
# Date: 2023.03.15

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
    rospy.loginfo("Successfully")

if __name__ == "__main__":
    rospy.init_node("dynamic_client")
    rospy.set_param("/mission/mic_awake", False)
    rospy.set_param("/mission/park_ready", False)
    parking = False
    rate = rospy.Rate(5)
    rospy.logwarn("______dynamic_readying__________________")
    # while not rospy.is_shutdown():
    #     mic_awake = rospy.get_param("/mission/mic_awake")
    #     if mic_awake:
    #         break
    #     rate.sleep()
    client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS", timeout = 10, config_callback = Callback)
    # client1 = dynamic_reconfigure.client.Client("/move_base/global_costmap", timeout = 10, config_callback = Callback)
    # client2 = dynamic_reconfigure.client.Client("/move_base/local_costmap", timeout = 10, config_callback = Callback)
    
    while  not rospy.is_shutdown():
        # rospy.logwarn("______parking_readying__________________")
        parking  = rospy.get_param("/mission/park_ready")
        var = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout = None)
        x = var.pose.pose.position.x
        y = var.pose.pose.position.y
        if x > 0 and x < 2.8 and y > -1.5 and y < 0.15 and parking :
            #A parking
            client.update_configuration({"inflation_dist": 0.1})   #    0.015
            client.update_configuration({"min_obstacle_dist": 0.05})    # 0.01
            # client1.update_configuration({"inflation_radius": 0.03})    #    0.05
            # client2.update_configuration({"inflation_radius": 0.03})    #    0.05

            # client.update_configuration({"weight_inflation": 100})
            # client.update_configuration({"weight_acc_lim_x": 1})
            # client.update_configuration({"weight_acc_lim_x": 1})
            # client.update_configuration({"weight_acc_lim_theta": 2})
            # client.update_configuration({"acc_lim_x": 0.4})
            # client.update_configuration({"acc_lim_y": 0.4})
            # client.update_configuration({"acc_lim_theta": 2})
            # client.update_configuration({"max_vel_x": 1})
            # client.update_configuration({"max_vel_y": 1})
            # client.update_configuration({"xy_goal_tolerance": 0.03})
            client.update_configuration({"xy_goal_tolerance": 0.05})
            client.update_configuration({"yaw_goal_tolerance": 0.2})
            rospy.logwarn("____________最小距离设置完了__________________")
            
        # elif x > 2.2 and x < 4.0 and y > -5.5 and y < -2.5:
            #D:
            # client.update_configuration({"min_obstacle_dist": 0.4})
            # client.update_configuration({"costmap_obstacles_behind_robot_dist": 5})
        else :
            pass
