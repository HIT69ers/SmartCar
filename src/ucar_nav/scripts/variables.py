#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import re
import roslib
import rospy  
import actionlib  
import collections
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, Vector3 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  
from std_msgs.msg import Int32, UInt16
import math
from Utils import *
from judge_conditions_copy import *
from photo_and_qr.srv import Greeting

import dynamic_reconfigure.client
# import sys   #导入sys模块
# sys.path.append("/home/ucar/ucar_ws/src/ucar_cam/scripts/")
# from yolo_node import MyUtils
from std_srvs.srv import Empty

# 清除代价地图
def Clear_Costmap():
    rospy.wait_for_service("/move_base/clear_costmaps")
    try:
        greetings_client = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        res = greetings_client.call()
        rospy.loginfo("Clear costmap")
    except rospy.ServiceException as e:
        rospy.logwarn("Clear costmap failed: %s"%e)



class NavTest():  
    def __init__(self, mode, stop_mode): 
        client_teb = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
        client_gc_inf = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer") 
        # rospy.init_node('random_navigation')
        self.mode = mode
        self.stop_mode = stop_mode # 0:pid 1:停车码
        rospy.loginfo(">>>>thread.py启动成功")
        # 等待语音唤醒消息

        # rospy.wait_for_message("/mic/awake/angle", Int32, timeout = None)
        Clear_Costmap()
        rospy.sleep(1)

        # client_teb.update_configuration({'max_vel_x' : 0.7})
        #rospy.on_shutdown(self.shutdown) 
        # 在每个目标位置暂停的时间  
        self.rest_time = rospy.get_param("~rest_time", 0)  
        #self.detact_go_out_flag = 1 # 1运行 0结束
        #self.stop_cancel_fail = 1 # 1运行 0结束
        # 到达目标的状态  
        # goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',      
        #                'SUCCEEDED', 'ABORTED', 'REJECTED',  
        #                'PREEMPTING', 'RECALLING', 'RECALLED',  
        #                'LOST']  
        self.location = ""  
        self.locations = collections.OrderedDict()
        # self.locations['0'] = Pose( Point(0.2, -0.05, 0.000), Quaternion(0.000, 0.000, -0.01, 1))
        self.locations['A'] = Pose( Point(0.93, 1.3, 0.000), Quaternion(0.000, 0.000, 0.048, 1))
        self.locations['A1'] = Pose( Point(0.92, 1.3, 0.000), Quaternion(0.000, 0.000, 0.048, 1))
        # # self.locations[a_bridge_first] = Pose(Point(-0.0666,-0.0175,0.000), Quaternion(0.000, 0.000, 1,0.05)) # 上桥的A侧
        # # self.locations[a_bridge_last] = Pose(Point(-1.411,-0.105,0.000), Quaternion(0.000, 0.000, 1,0.06)) # 上桥的A侧
        self.locations['center'] = Pose(Point(-1.83,0.00, 0.000), Quaternion(0.000, 0.000,0.997,-0.0789)) #1.83
        # client_gc_inf.update_configuration({'inflation_radius' : 0.15})
        # client_teb.update_configuration({'max_vel_x' : 0.8})
        # self.locations['B'] = Pose(Point(-2.56, -1.86, 0.000),Quaternion(0.000, 0.000, -0.721, 0.7)) #0628
        # self.locations['B'] = Pose(Point(-2.46, -1.76, 0.000),Quaternion(0.000, 0.000, -0.721, 0.7)) #0707
        self.locations['B'] = Pose(Point(-2.62, -1.85, 0.000),Quaternion(0.000, 0.000, -0.721, 0.7)) #0708
        self.locations['B1'] = Pose(Point(-2.56, -1.84, 0.000),Quaternion(0.000, 0.000, -0.721, 0.7))  #-2.56
        #  self.locations['rotate1'] = Pose(Point(-1.83,-1.15, 0.000), Quaternion(0.000, 0.000, 0.346,0.94)) 
        # self.locations['rotate3'] = Pose(Point(-1.83,0, 0.000), Quaternion(0.000, 0.000, 0.346,0.94))
        #  self.locations['rotate2'] = Pose(Point(-1.827,1.008, 0.000), Quaternion(0.000, 0.000, -0.95,0.315)) 
        self.locations[f_bridge_first] = Pose(Point(-1.9,0.03, 0.000), Quaternion(0.000, 0.000, 0,1)) # 下桥的F侧  7-7 by zyx
        # self.locations[f_bridge_first] = Pose(Point(-1.9,0.03, 0.000), Quaternion(0.000, 0.000, 0,1)) # 下桥的F侧  7-7 by zyx
        # self.locations[f_bridge_last] = Pose(Point(0.01,-0.06, 0.000), Quaternion(0.000, 0.000, 0.01,1)) # 下桥的F侧
        # self.locations[f_bridge_last] = Pose(Point(0.3,-0.06, 0.000), Quaternion(0.000, 0.000, 0.01,1)) # 下桥的F侧，导航停车
        # # self.locations[start_t] = Pose(Point(-0.1513, -0.25, 0.000),Quaternion(0.000, 0.000, -0.54, 0.84))
        # self.locations[start_p] = Pose(Point(1.80, -0.05, 0.000),Quaternion(0.000, 0.000, 0.01 ,1))
        # self.locations[start_p] = Pose(Point(1.75, -0.05, 0.000),Quaternion(0.000, 0.000, 0 ,1))
        
        
        
        #新地图新点
        # self.locations['A'] = Pose( Point(0.8716, 1.11, 0.000), Quaternion(0.000, 0.000, -0.035, 1))
        # self.locations['A1'] = Pose( Point(0.86, 1.11, 0.000), Quaternion(0.000, 0.000, -0.035, 1))
        # self.locations['B'] = Pose(Point(-2.4277, -1.72, 0.000),Quaternion(0.000, 0.000, -0.744, 0.668))
        # self.locations['B1'] = Pose(Point(-2.437, -1.72, 0.000),Quaternion(0.000, 0.000, -0.744, 0.668))
        # self.locations['rotate1'] = Pose(Point(-2,-1.4, 0.000), Quaternion(0.000, 0.000, 0.338,0.94)) 
        # self.locations['rotate3'] = Pose(Point(-1.856,-0.266, 0.000), Quaternion(0.000, 0.000, 0.364,0.93))
        # self.locations['rotate2'] = Pose(Point(-2.05,0.9, 0.000), Quaternion(0.000, 0.000, -0.3,0.95)) 
        # self.locations[f_bridge_first] = Pose(Point(-1.78,0.044, 0.000), Quaternion(0.000, 0.000, -0.012,1)) 
    

        self.locations_keys = list(self.locations.keys())

        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction) 

        #self.cancel_mark = 0  # 1为已经取消掉
        self.client_gc = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
        # 调inflation_radius需要用inflation_layer
        self.client_lc = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer") 
        self.client_teb = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
        self.client_gp = dynamic_reconfigure.client.Client("/move_base/GlobalPlanner")
        self.boards_1 = []
        #self.boards_2 = []

        Clear_Costmap()


    def navigation(self): 
        rospy.loginfo("Waiting for move_base action server...")  
        flag = {'A':0, 'B':0,'B1':0,'B2':0,'B3':0, 'C':0}
        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  

        # 保存机器人的在rviz中的初始位置  
        initial_pose = PoseWithCovarianceStamped()  

        # 保存成功率、运行时间、和距离的变量  
        n_locations = len(self.locations)  
        i = n_locations  
        distance_traveled = 0  
        self.start_time = rospy.Time.now().to_sec()
        rospy.set_param('start_time', self.start_time)
        running_time = 0  
        # 确保有初始位置  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  

        rospy.loginfo("Starting navigation test") 
        # rospy.set_param("has_reached_B",1)
        print(self.locations_keys)
        while len(self.locations_keys) > 0:
        # for self.location in self.locations_keys:
            self.location = self.locations_keys.pop(0)
            rospy.set_param("location_in_f", self.location)
            print(self.locations_keys)
            
            self.cancel_mark = 0 # 对于每个目标点，都要重新设置为0，两种取消情况取消掉后，都要置为1
            
            rospy.loginfo("Updating current pose.")
            initial_pose.header.stamp = "" 
 

            # 设定下一个目标点  
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = self.locations[self.location]  
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  

            # 让用户知道下一个位置  
            rospy.loginfo("Going to: " + str(self.location))  
            # 向下一个位置进发  
            self.move_base.send_goal(self.goal)  
            # if (rospy.get_param('rescue', 0) == 4 and self.location.startswith('rotate')) or (str(self.location)=='m_board1'):
            if (rospy.get_param('rescue', 0) == 4 and (self.location.startswith('rotate')  or (str(self.location)=='m_board0'))):  
                # 先找1后找0  
                self.move_base.cancel_goal()
                rospy.loginfo("跳过"+ self.location)
                self.cancel_mark =1
            
            if self.location==f_bridge_first:
                if  rospy.get_param('rescue',  0) != 4:
                    self.cancel_mark = 1
                    insertnew(self)
            
            # 时间限制
            if self.location.startswith ('mf_bridge'):
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
            else:
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(20))

            if not finished_within_time:
                # insertnew(self)
                if str(self.location)=='m_board0' or str(self.location)=='m_board1':
                    print(f"rescue={rospy.get_param('rescue',0)}")
                    if rospy.get_param('rescue',0)==3:
                        rospy.set_param('fail2get', 1)
                        rospy.set_param('nongoal_flag', 6)#0712
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            elif self.cancel_mark != 1: # 目标点没有被取消掉
                rospy.loginfo("未被取消的抵达" + self.location)
                rotate_shot_strategy(self)
                print(f"findboard={rospy.get_param('findboard', 0)}")

                if(rospy.get_param('findboard', 0) == 1):
                    shot_concise(self)
                    rospy.set_param('findboard', 0)
                if (str(self.location)=='m_board0' or str(self.location)=='m_board1' )and rospy.get_param('rescue',  0) == 3:
                    rospy.set_param('get_board', 1)
                    print(f"rescue={rospy.get_param('rescue',0)}")
                    print(f"rospy.get_param('flag_findornot_nav', 0) {rospy.get_param('flag_findornot_nav', 0)}")
                if  (rospy.get_param('flag_findornot_nav', 0) == 0 or (rospy.get_param('get_board',  0) == 1) ) and rospy.get_param('rescue',  0) != 4:      #完成导航找板子or rospy.get_param('flag_PID2',  0)==1,rospy.get_param('findboard',  0) == 2 or 
                    rospy.set_param('nongoal_flag', 6)
                if  rospy.get_param('rescue',  0) == 4:
                    rospy.sleep(1.5)
                    rospy.set_param('nongoal_flag', 0)
                pidA2F(self)
            print(f"set_goal2={rospy.get_param('set_goal',0)}")
            print(f"nongoal_flag={rospy.get_param('nongoal_flag',0)}")
            
            if str(self.location)=='center' :
                client_teb.update_configuration({'max_vel_x' : 0.7})
            if str(self.location)=='f_bridge_first' :
                client_teb.update_configuration({'xy_goal_tolerance' : 0.08})
            if str(self.location)=='start_p' :
                client_teb.update_configuration({'max_vel_x' : 0.5})
                client_teb.update_configuration({'max_vel_y' : 0.0001})
                client_teb.update_configuration({'max_vel_x_backwards' : 0.0001})
                client_teb.update_configuration({'min_turning_radius' : 0.08})
                client_teb.update_configuration({'acc_lim_x' : 0.5})
                client_teb.update_configuration({'yaw_goal_tolerance' : 0.08})
                client_gc_inf.update_configuration({'inflation_radius' : 0.25})
            # if rospy.get_param('flag_findornot_nav',0) == -1:
            if (str(self.location)=='center'  and rospy.get_param('set_goal',0)==2) or (rospy.get_param('fail2get', 0)==1 and rospy.get_param('first_aid_kit', 0)!=1 and rospy.get_param("direction", 0)==2): #试探策略
                self.locations['rotate2'] = Pose(Point(-1.827,0.95, 0.000), Quaternion(0.000, 0.000, -0.95,0.315)) 
                self.locations_keys.insert(0,'rotate2')
            elif (str(self.location)=='center'  and rospy.get_param('set_goal',0)==3) or (rospy.get_param('fail2get', 0)==1 and rospy.get_param('first_aid_kit', 0)!=1 and rospy.get_param("direction", 0)==3):
                self.locations['rotate1'] = Pose(Point(-1.83,-1.15, 0.000), Quaternion(0.000, 0.000, 0.346,0.94)) 
                self.locations_keys.insert(0,'rotate1')
            # else:
            if (str(self.location)=='m_board0' or str(self.location)=='m_board1' )and rospy.get_param('rescue',0)!=3: #试探策略
                self.move_base.cancel_goal()

            if self.location.startswith('A') and flag['A']==0 :
                flag['A'] = 1
                print("A")
                rospy.set_param('take_photo', 1)
                while True:
                    if(rospy.get_param('take_photo',  1) == 0 ):
                        break
            if str(self.location)=='B1'  and flag['B']==0:
                flag['B'] = 1
                print("B")
                rospy.set_param('first_aid_kit', 1)
                while True:
                    if(rospy.get_param('first_aid_kit',  1) == 0 ):
                        break  
            if self.location==f_bridge_last:
                if rospy.get_param('trace_line')==0:
                    rospy.set_param('nongoal_flag', 0)  
                else:
                    rospy.set_param('nongoal_flag', 5)  
            if self.location==start_p:
                rospy.set_param('ready_stop', 1)  
            if self.location=='0':
               Clear_Costmap()
            
        # rospy.spin()
        print('navigation finished!')
        
    def update_initial_pose(self, initial_pose): 
        self.initial_pose = initial_pose  
       

    def rotate(self, angular_speed, goal_angle):
        """
        @param goal_angle:想要旋转的角度（角度制）
        """
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = angular_speed
        duration = math.radians(goal_angle) / abs(angular_speed)
        begin_time = rospy.Time.now()
        while True:
            self.cmd_vel_pub.publish(twist)
            if int((rospy.Time.now() - begin_time).to_sec() / duration) >= 1:
                break

    def Client_Srv(self):
        rospy.wait_for_service("greetings")
        try:
            greetings_client = rospy.ServiceProxy("greetings", Greeting)
            res = greetings_client.call(1)
            rospy.loginfo("Server call: %s"%res.feedback)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)