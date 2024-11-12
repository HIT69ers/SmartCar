#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from variables import Clear_Costmap
def correct_initial() :
    rate = rospy.Rate(5)

    initial_pose =  rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout= None)
    # initial_pose = PoseWithCovarianceStamped()
    initial_pose.pose.pose.position.x = 0.05
    initial_pose.pose.pose.position.y = 0.05
    initial_pose.pose.pose.position.z = 0
    initial_pose.pose.pose.orientation.x = 0
    initial_pose.pose.pose.orientation.y = 0
    initial_pose.pose.pose.orientation.z = 0
    initial_pose.pose.pose.orientation.w = 1

    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = 'map'
    
    count = 0
    while not rospy.is_shutdown(): 
        initial_pose.header.stamp = rospy.Time.now()    
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        pub.publish(initial_pose)
        Clear_Costmap()
        count += 1
        if count > 5:
            break
        rate.sleep()