#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import rospy  
from actionlib_msgs.msg import *  
from random import sample  
from math import pow, sqrt  
from std_msgs.msg import Int32
import threading
from variables import *
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from Utils import *
from judge_conditions import reach_B 
from rosgraph_msgs.msg import Log
import rosnode

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
    if ((former !=1 and latter == 1) or count_nonlegal > 50) and navtest.location != f_bridge_second and navtest.location != 'rotate_p' : #and  navtest.location != a_bridge_first :    # aTobridge(?) 好像直接去F也可以(?)
        rospy.loginfo("have canceled the goal!!!!")
        count_nonlegal = 0 # 取消后，计数器清零
        
        if navtest.location == a_bridge_first or navtest.location == 'start_p':
            navtest.cancel_mark = 0
            Clear_Costmap()
        else:
            navtest.cancel_mark = 1 # 取消标志置1
            Clear_Costmap()
            navtest.move_base.cancel_goal()

def trunc(f, n):   
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  

def multi_goal_nav(navtest):
    try:  
        navtest.navigation()
        # rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("Random navigation finished.")

def receive_islegal(navtest):
    """
        判断是否出现 Failed to get a plan 的情况, 如果出现, 则取消目标点
        取消目标点的策略在回调函数doMsg中实现
    """
    sub = rospy.Subscriber("/move_base/GlobalPlanner/cxy_legal",Int32,doMsg,callback_args=(navtest,),queue_size=10)
    while True:
        rospy.sleep(0.2)
        if navtest.stop_cancel_fail == 0:
            break
    print('receive is legal has finished')


hard_points = ['E1','E2','E3','E4','D1','D2','D3','D4','C1','C2','C3','C4']

def doCancelHardPoints(msg, args):
    """
    硬点取消回掉函数
    """
    # 如果 目标点在想取消的点集之中 且 当前位置距离目标点很近
    navtest = args[0]
    if navtest.location in hard_points and nearby_2d((msg.position.x, msg.position.y), navtest.locations[navtest.location], 0.2):
        rospy.loginfo("硬点"+ navtest.location +"被取消")
        navtest.cancel_mark = 1
        navtest.move_base.cancel_goal()


def cancel_hard_points():
    """
    硬点取消函数
    """
    sub = rospy.Subscriber("/cxy_base_link", Pose, doCancelHardPoints, queue_size=1)
    rospy.spin()

# def callback_rosout(data):
#     global unfeasible_times
#     if "not feasible" in data.msg:
#         unfeasible_times += 1
#         if unfeasible_times >= 3 and ((navtest.location[0] in navtest.has_detected) or (navtest.location[1] != '4')) and navtest.location != f_bridge_second and navtest.location != 'rotate_p' :  # question： 次数太少会不停地跳点，次数太多会一直向不可能到达的点规划而不failed
#             navtest.move_base.cancel_goal()
#             Clear_Costmap()
#             rospy.logwarn("have canceled the goal!!!!")
#             count_nonlegal = 0 # 取消后，计数器清零
#             navtest.cancel_mark = 1 # 取消标志置1
#             unfeasible_times = 0
#     else:
#         unfeasible_times = 0

# def receive_unfeasible():
#     """
#     判断是否出现TebLocalPlannerROS: trajectory is not feasible. Resetting planner..的警告
#     针对不能到达目标点但在反复进行规划的情况(无Failed to get a plan)
#     """
#     sub = rospy.Subscriber("/rosout", Log, callback_rosout)
#     rospy.spin()

def detect_go_out(navtest):
    while True:
        rospy.sleep(0.01)
        if navtest.detact_go_out_flag == 0:
            break
        # 房间已经识别出来，并且确保是当前正试图达到的目标点所在的房间
        if rospy.get_param('room_detected', 0) == 1 and rospy.get_param('room_id', 'non_id') == navtest.location[0]:
            rospy.set_param('room_detected', 0)
            navtest.has_detected.append(navtest.location[0]) #该房间已经抵达，加入已抵达列表
            navtest.move_base.cancel_goal()
            rospy.loginfo(navtest.location +" has been detected!!!")
            reach_B(navtest)
            navtest.cancel_mark = 1
    print('detect go out has finished')

def main(mode,stop_mode):
    navtest = NavTest(mode,stop_mode)
    thread1 = threading.Thread(target=multi_goal_nav, args=(navtest,))
    thread1.start()
    thread2 = threading.Thread(target=receive_islegal, args=(navtest,))
    thread2.start()
    thread3 = threading.Thread(target=detect_go_out, args=(navtest,))
    thread3.start()
    # thread4 = threading.Thread(target=cancel_hard_points)
    # thread4.start()

if __name__ == '__main__':
    print("thread等待启动")
    # stop_event = threading.Event()
    rospy.init_node('random_navigation')
    
    while not rospy.is_shutdown():
        rospy.sleep(0.2)
        
        if rospy.get_param('nav_modified', 0) == 1:
            mode = rospy.get_param('mode', 0)
            stop_mode = rospy.get_param('stop_mode', 0)
            print(f"mode is {mode}")
            print(f"stop_mode is {mode}")
            main(mode, stop_mode)
            rospy.set_param('nav_modified', 0)
            # print("thread等待启动")
            
    # navtest = NavTest(0)
    # thread1 = threading.Thread(target=multi_goal_nav, args=(navtest,))
    # thread1.start()
    # thread2 = threading.Thread(target=receive_islegal, args=(navtest,))
    # thread2.start()
    # thread3 = threading.Thread(target=detect_go_out, args=(navtest,))
    # thread3.start()
    
   
