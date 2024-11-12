#!/usr/bin/env python
# coding:utf-8

# 加载所需模块
import rospy
from photo_and_qr.srv import Greeting

# from PhotoAndQR.srv import *

# def Client_Srv():
#     rospy.init_node('PhotoAndQR_Client')
#     rospy.wait_for_service("greetings")
#     try:
#         greetings_client = rospy.ServiceProxy("greetings", Greeting)
#         response = PhotoAndQR_Client.call(position, num, flag)
#         rospy.loginfo("Message From server:%s"%response.feedback)
#     except rospy.ServiceException as e:
#         rospy.logwarn("Service call failed: %s"%e)

def Client_Srv():
    rospy.wait_for_service("greetings")
    try:
        greetings_client = rospy.ServiceProxy("greetings", Greeting)
        res = greetings_client.call(1)
        rospy.loginfo("Server call: %s"%res.feedback)
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s"%e)

if __name__=="__main__":
    Client_Srv()
