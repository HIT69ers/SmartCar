#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import rospy  
from actionlib_msgs.msg import *  
from random import sample  
from math import pow, sqrt  
from std_msgs.msg import Int32
import threading
from variables import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from Utils import *
from judge_conditions import reach_B 
from rosgraph_msgs.msg import Log

former = -1
latter = -1  
count_nonlegal = 0

def doMsg(msg, args):
    """
        取消目标点的回调函数
        
        正常时/move_base/GlobalPlanner/cxy_legal 发布的值是0, 不正常时发布的是1
        通过比较former和latter的值, 如果:
        1.former !=1 and latter == 1
        2.count_nonlegal > 50 (针对连续fail的情况)
        则将目标取消掉
    """
    global former, latter, count_nonlegal
    navtest = args[0]
    former = latter
    latter = msg.data
    if latter == 1:
        count_nonlegal += 1
    if ((former !=1 and latter == 1) or count_nonlegal > 30) and  navtest.location != 'rotate_p' : #and  navtest.location != a_bridge_first :    # aTobridge(?) 好像直接去F也可以(?)
        rospy.loginfo("have canceled the goal!!!!")
        count_nonlegal = 0 # 取消后，计数器清零
        insertnew(navtest)
        if navtest.location.startswith('m_board') or navtest.location == 'A' or navtest.location == 'A1':# or navtest.location =='center'//0628
            navtest.cancel_mark = 0
            Clear_Costmap()
            navtest.move_base.cancel_goal()
        else:
            navtest.cancel_mark = 1 # 取消标志置1
            Clear_Costmap()
            navtest.move_base.cancel_goal()

def receive_islegal(navtest):
    """
        判断是否出现 Failed to get a plan 的情况, 如果出现, 则取消目标点
        取消目标点的策略在回调函数doMsg中实现
    """
    sub = rospy.Subscriber("/move_base/GlobalPlanner/cxy_legal",Int32,doMsg,callback_args=(navtest,),queue_size=10)
    # while True:
    #     rospy.sleep(0.2)
    #     if navtest.stop_cancel_fail == 0:
    #         break
    rospy.spin()
    print('receive is legal has finished')

def multi_goal_nav(navtest):
    try:  
        navtest.navigation()
        # rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("Random navigation finished.")

hard_points = ['0','A','A1','rotate1','rotate10','rotate11','rotate111','rotate2','rotate20','rotate22','rotate222','B','B1','mf_bridge_last','start_p']

def doCancelHardPoints(msg, args):
    """
    硬点取消回掉函数
    """
    # 如果 目标点在想取消的点集之中 且 当前位置距离目标点很近
    navtest = args[0]
    rospy.set_param("cancel",1)
    if navtest.location in hard_points and nearby_2d((msg.position.x, msg.position.y), (navtest.locations[navtest.location].position.x,navtest.locations[navtest.location].position.y), 0.4):
        if navtest.location.startswith('rotate'):                                                                                                                                                               #旋转点
            if not navtest.move_base.wait_for_result(rospy.Duration(5)):
                navtest.cancel_mark = 0
                navtest.move_base.cancel_goal()
                rospy.loginfo("硬点"+ navtest.location +"被取消")

        elif navtest.location== start_p :                                                                                                                                                                                #导航停车点
            if nearby_2d((msg.position.x, msg.position.y), (navtest.locations[navtest.location].position.x,navtest.locations[navtest.location].position.y), 0.3):
                rospy.set_param('nongoal_flag', 7)  
                navtest.cancel_mark = 1
                navtest.move_base.cancel_goal()
                if rospy.get_param('ready_stop', 0)==0:
                    return 0
        # elif navtest.location.startswith('m_board'):                                                                                                                                                           #导航取板点
        #     if nearby_2d((msg.position.x, msg.position.y), (navtest.locations[navtest.location].position.x,navtest.locations[navtest.location].position.y), 0.1):
        #         navtest.cancel_mark = 0
        #         navtest.move_base.cancel_goal()

        elif nearby_2d((msg.position.x, msg.position.y), (navtest.locations[navtest.location].position.x,navtest.locations[navtest.location].position.y), 0.07):
            if navtest.location==f_bridge_last:
                if rospy.get_param('trace_line')==0:
                    rospy.set_param('nongoal_flag', 0)  
                else:
                    rospy.set_param('nongoal_flag', 5)
            navtest.cancel_mark = 1
            navtest.move_base.cancel_goal()
            rospy.loginfo("硬点"+ navtest.location +"被取消")





def cancel_hard_points():
    """
    硬点取消函数
    """
    sub = rospy.Subscriber("/cxy_base_link", Pose, doCancelHardPoints, callback_args=(navtest,),queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    #print("thread等待启动")
    rospy.init_node('random_navigation')
    while not rospy.is_shutdown():
        rospy.sleep(0.2)

        if rospy.get_param('nav_modified', 0) == 1:
            navtest = NavTest(0,0)
            thread1 = threading.Thread(target=multi_goal_nav, args=(navtest,))
            thread1.start() 
            thread2 = threading.Thread(target=receive_islegal, args=(navtest,))
            thread2.start()
            thread3 = threading.Thread(target=cancel_hard_points)
            thread3.start()
            rospy.set_param('nav_modified', 0)