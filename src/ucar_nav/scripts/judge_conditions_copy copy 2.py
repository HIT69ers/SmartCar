#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import rospy
from Utils import *
from findboard import BoardNav
from nav_msgs.msg import OccupancyGrid
import cv2
import dynamic_reconfigure.client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from tf.transformations import euler_from_quaternion

SHOT_EDGE = 1
SHOT_IN = 2

client_lc_inf = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer") 
client_lc = dynamic_reconfigure.client.Client("/move_base/local_costmap") 
client_gc = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer")
client_teb = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
client_gc_inf = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer") 
# client_gc_ob = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer") 
# client_lc_ob = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer") 
def reach_B(nav):
    """
    判断是否已经到达B,置位has_reached_B参数
    0:未到达B房间,1:到达B房间
    """
    if nav.location.startswith('B') and 'B' in nav.has_detected:
        rospy.loginfo("I have reached B!")
        rospy.set_param("has_reached_B",1)


def shot_judge(nav, head):
    """
        提供拍照标记位
        @params head: 当前location以head开头时,可以拍照
    """
    if nav.location.startswith(head):
        rospy.set_param("can_shot", 1)
        rospy.loginfo(">>> shot at " + nav.location)
        rospy.sleep(1)
        rospy.loginfo(">>>go")

def amcl_callback(msg):
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()
        if msg.position.x < 1:
            break

def reached_set(nav):
    tmp_num = nav.location[1]
    if tmp_num == '2':
        rospy.set_param('shot_start_point', 2)
        print('set 2')
    elif tmp_num == '3':
        rospy.set_param('shot_start_point', 3)
        print('set 3')

    elif tmp_num == '4':
        rospy.set_param('shot_start_point', 4)
        print('set 4')

    elif tmp_num == '5':
        rospy.set_param('shot_start_point', 5)
        print('set 5')



def start_condition_judge_5(nav):
    """适用于5点和旋转"""
    if nav.location == a_bridge_first:
        rospy.set_param("has_reached_F",1)
        nav.detact_go_out_flag = 0
        client_teb.update_configuration({'xy_goal_tolerance' : 0.05})
        client_teb.update_configuration({'yaw_goal_tolerance' : 0.2})
    elif nav.location == 'rotate_p':
        nav.client_teb.update_configuration({'xy_goal_tolerance' : 0.20})       # 0.35
        nav.client_teb.update_configuration({'yaw_goal_tolerance' : 0.08})      # 0.18
    # elif nav.location == 'start_p':
    #     # odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)
    #     rate = rospy.Rate(5)
    #     while not rospy.is_shutdown():
    #         cur_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout = 1000)
    #         # rospy.loginfo("x = %f\n", cur_pose.pose.pose.position.x)
    #         # if cur_pose.pose.pose.position.x < 2.0:  
    #         if cur_pose.pose.pose.position.y > -0.7:  
    #             break
    #         rate.sleep()
    #     client_gc.update_configuration({'inflation_radius' : 0.05})
    #     client_gc.update_configuration({'cost_scaling_factor' : 75})
    #     client_lc_inf.update_configuration({'inflation_radius' : 0.05})
    #     client_teb.update_configuration({'min_obstacle_dist' : 0.0})
    #     client_teb.update_configuration({'inflation_dist' : 0.0})
    #     nav.client_teb.update_configuration({'xy_goal_tolerance' : 0.05})
    #     rospy.set_param('/move_base/TebLocalPlannerROS/footprint_model/type', 'point')
    #     # rospy.set_param('/move_base/TebLocalPlannerROS/footprint_model/vertices', [[0.01, -0.01], [0.01, 0.01],[-0.01, 0.01], [-0.01, -0.01]])
    #     return 1
    
    elif nav.location == 'm_left_down':
        nav.client_teb.update_configuration({'xy_goal_tolerance' : 0.05})
        nav.client_teb.update_configuration({'yaw_goal_tolerance' : 0.1})
    elif nav.location == 'm_right_down':
        pass
        # client_gc_ob.update_configuration({'raytrace_range': 2.5})
        # client_lc_ob.update_configuration({'raytrace_range': 2.5})
    elif nav.location.startswith('G'):
        # client_gc_ob.update_configuration({'raytrace_range': 0.0})
        # client_lc_ob.update_configuration({'raytrace_range': 0.0})
        nav.client_teb.update_configuration({'xy_goal_tolerance' : 0.25})      
        nav.client_teb.update_configuration({'yaw_goal_tolerance' : 0.18})     
    elif nav.location == 'start_p':
        nav.client_teb.update_configuration({'xy_goal_tolerance' : 0.05})
        nav.client_teb.update_configuration({'yaw_goal_tolerance' : 0.1})
    elif nav.location.startswith('m_board'):
        nav.client_teb.update_configuration({'xy_goal_tolerance' : 0.3})
        nav.client_teb.update_configuration({'yaw_goal_tolerance' : 0.3})

def shot_concise(nav):
    """ 控制开始雷达扫描板子导航"""
    # 抵达G区域内后
    if nav.location == 'G1':
        boards = get_boards()
        nav.boards_1 = boards
        if len(boards) == 2:
            nav.has_reached_G = True
            if boards[0].uni_shot: # 只拍一张 标记位置为board0/1
                nav.locations['m_board0'] = boards[0].aim_pose
                nav.locations_keys.insert(0,'m_board0')
            elif boards[1].uni_shot:
                nav.locations['m_board1'] = boards[1].aim_pose
                nav.locations_keys.insert(0,'m_board1')
            else:
                for i,board in enumerate(boards): # 拍两张
                    nav.locations['m_board'+str(i)] = board.aim_pose
                    nav.locations_keys.insert(0,'m_board'+str(1 - i))
        elif len(boards) == 1: # 只拍一张 标记位置为board3
            nav.has_reached_G = True
            nav.locations['m_board3'] = boards[0].aim_pose
            nav.locations_keys.insert(0,'m_board3')
        elif len(boards) == 3:
            nav.has_reached_G = True
            for i,board in enumerate(boards): # 拍三张
                nav.locations['m_board'+str(i)] = board.aim_pose
                nav.locations_keys.insert(0,'m_board'+str(2- i))

    if nav.location == 'G2':
        boards = get_boards()
        if len(boards) == 2:
            nav.has_reached_G = True
            if boards[0].uni_shot: # 只拍一张 标记位置为board0/1
                nav.locations['m_board0'] = boards[0].aim_pose
                nav.locations_keys.insert(0,'m_board0')
            elif boards[1].uni_shot:
                nav.locations['m_board1'] = boards[1].aim_pose
                nav.locations_keys.insert(0,'m_board1')
            else:
                for i,board in enumerate(boards): # 拍两张
                    nav.locations['m_board'+str(i)] = board.aim_pose
                    nav.locations_keys.insert(0,'m_board'+str(1 - i))
        elif len(boards) == 1: # 只拍一张 标记位置为board3
            nav.has_reached_G = True
            nav.locations['m_board3'] = boards[0].aim_pose
            nav.locations_keys.insert(0,'m_board3')
        elif len(boards) == 3:
            nav.has_reached_G = True
            for i,board in enumerate(boards): # 拍三张
                nav.locations['m_board'+str(i)] = board.aim_pose
                nav.locations_keys.insert(0,'m_board'+str(2 - i))

    if nav.location.startswith('m_board'):
        rospy.set_param("can_shot", 2) # 内为2

    if nav.location == 'm_right_down':
        rospy.set_param("can_shot", 1)
        rospy.loginfo(">>> shot right ")
        rospy.sleep(0.5)

        nav.rotate(1, 120)
        rospy.set_param("can_shot", 1)
        rospy.loginfo(">>> shot right ")
        rospy.sleep(0.5)

    if nav.location == 'm_left_down':
        #左下拍
        rospy.set_param("can_shot", 1)
        rospy.loginfo(">>> shot left ")
        rospy.sleep(0.5)
        nav.rotate(-1, 70)

        #左上拍
        rospy.set_param("can_shot", 1)
        rospy.loginfo(">>> shot left ")
        rospy.sleep(0.5)
        nav.rotate(1, 35)

        # pid F->A 过桥
        rospy.set_param('nongoal_flag', 2)
        rospy.set_param("has_reached_F",2)
        nav.stop_cancel_fail = 0
        

def rotatePic(data1):
    data = []
    for x in data1:
        if x <0:
            data.append(0)
        else:
            data.append(x)

    data = np.array(data)
    data = data.reshape(250, -1)
    image = data.astype(np.uint8)
    height, width = image.shape[:2]
    center = (width/2, height/2)
    rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=6, scale=1)
    rotated_image = cv2.warpAffine(src=image, M=rotate_matrix, dsize=(width, height))
    return rotated_image

def gen_cutPic(data1):
    def rotatePic(data1):
        data = []
        for x in data1:
            if x <0:
                data.append(0)
            else:
                data.append(x)

        data = np.array(data)
        # data = data.reshape(250, -1)
        data = data.reshape(235, -1)

        image = data.astype(np.uint8)
        height, width = image.shape[:2]
        center = (width/2, height/2)
        rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=1, scale=1)
        rotated_image = cv2.warpAffine(src=image, M=rotate_matrix, dsize=(width, height))
        return rotated_image
    rotated_image = rotatePic(data1)
    # 第一个值控制上下 第二个值控制左右
    leftDown = (111,66) # [0] 变大可以让车更往F房间深处走
    sideLength = 40
    cutOutPic = np.zeros((sideLength, sideLength)).astype(np.uint8)
    for x in range(sideLength):
        for y in range(sideLength):
            cutOutPic[x][y] = rotated_image[leftDown[0]+x][leftDown[1]+y]
    with open('/home/ucar/ucar_ws/src/ucar_nav/scripts/board_data.py','w') as f:
        f.write('data = ' + str(cutOutPic.tolist()))
    return cutOutPic

def get_boards():
    boardnav = BoardNav()
    rospy.loginfo('>>>shot from laser info')
    position = rospy.wait_for_message('/cxy_base_link', Pose, timeout = 5)
    boardnav.now_position = (position.position.x, position.position.y)
    data = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout = 5)
    rospy.loginfo((position.position.x, position.position.y))
    boardnav.resolution = data.info.resolution
    boardnav.img = gen_cutPic(data.data)
    boards = boardnav.findboard() #当找到两个板子时，令has_reached_G为true
    return boards




def insertGnew(nav):
    if nav.location == a_bridge_first:
        # F内3*3区域的取点 不妨认为其为房间G
        nav.locations['m_left_down'] = Pose(Point(1.000, -2.722, 0.000), Quaternion(0.000, 0.000, 0.874, 0.486))  # define
        nav.locations['m_right_down'] = Pose(Point(1.020, -4.861, 0.000), Quaternion(0.000, 0.000, -0.946, 0.324))
        nav.locations['G1'] = Pose(Point(1.009, -2.895, 0.000), Quaternion(0.000, 0.000, -0.674, 0.739))
        nav.locations['G2'] = Pose(Point(0.409, -2.895, 0.000), Quaternion(0.000, 0.000, -0.674, 0.739))
        # nav.locations['G3'] = Pose(Point(1.221, -4.255, 0.000), Quaternion(0.000, 0.000, -0.674, 0.739))
        nav.locations_keys.insert(0,'m_left_down')
        nav.locations_keys.insert(0,'m_right_down')
        # nav.locations_keys.insert(0,'G3')
        nav.locations_keys.insert(0,'G2')
        nav.locations_keys.insert(0,'G1')
        
def get_turn_list(now_angle, shot_angle_list):
    """获取旋转角度列表"""
    shot_angle_list.sort(key=sort_by_angle)
    # print(angle_list)
    turn_list = []
    for Angle in shot_angle_list:
        if Angle.angle <= now_angle and Angle.angle > 0:
            turn_list.append(ShotAngle(now_angle - Angle.angle, Angle.shot_type))
        elif Angle.angle <= 0:
            turn_list.append(ShotAngle(now_angle - Angle.angle, Angle.shot_type))
        elif Angle.angle > now_angle:
            turn_list.append(ShotAngle(now_angle + 180 + 180 - Angle.angle, Angle.shot_type))
    turn_list.sort(key=sort_by_angle)
    # print(turn_list)
    diff_turn_list = []
    diff_turn_list.append(turn_list[0])
    for i in range(len(turn_list) - 1):
        diff_turn_list.append(ShotAngle(turn_list[i + 1].angle - turn_list[i].angle, turn_list[i + 1].shot_type))
    # print(diff_turn_list)
    return diff_turn_list

def combined(nav):
    if nav.location == 'G1':
        # boardnav
        boardnav = BoardNav()
        rospy.loginfo('>>>shot from laser info')
        position = rospy.wait_for_message('/cxy_base_link', Pose, timeout = 5)
        boardnav.now_position = (position.position.x, position.position.y)
        data = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout = 5)
        rospy.loginfo((position.position.x, position.position.y))
        boardnav.resolution = data.info.resolution
        boardnav.img = gen_cutPic(data.data)
        boards = boardnav.rotate_strategy()
        nav.boards_1 = boards
        occupied = False
        for board in boards:
            # 障碍板是否在中央点附近
            if point_in_convex_polygon(board.position, boardnav.get_central_square().flatten().tolist()):
                print('旋转中央点被占据')
                occupied = True
                break
        if not occupied: # 没被占
            print('旋转中央点wei被占据')
            if nav.mode == 2:
                nav.locations_keys.pop(0) # 弹出G2
                nav.locations_keys.pop(0) # 弹出m_right_down
                nav.locations_keys.pop(0) # 弹出m_left_down
                nav.locations['m_center'] = Pose(Point(1.009, -3.395, 0.000), Quaternion(0.000, 0.000, 0.439, 0.898))
                nav.locations_keys.insert(0,'m_center')
            if nav.mode == 1:
                nav.locations_keys.pop(0) # 弹出G2
                nav.locations_keys.pop(0) # 弹出m_right_down
                nav.locations_keys.pop(0) # 弹出m_left_down
                # nav.locations['m_center'] = Pose(Point(1.009, -3.395, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
                nav.locations['m_center'] = Pose(Point(1.009, -3.395, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
                nav.locations_keys.insert(0,'m_center')
            if nav.mode == 0:
                # nav.locations_keys.pop(0) # 弹出G2
                boards = boardnav.findboard()
                if len(boards) == 2:
                    nav.has_reached_G = True
                    if boards[0].uni_shot: # 只拍一张 标记位置为board0/1
                        nav.locations['m_board0'] = boards[0].aim_pose
                        nav.locations_keys.insert(0,'m_board0')
                    elif boards[1].uni_shot:
                        nav.locations['m_board1'] = boards[1].aim_pose
                        nav.locations_keys.insert(0,'m_board1')
                    else:
                        for i,board in enumerate(boards): # 拍两张
                            nav.locations['m_board'+str(i)] = board.aim_pose
                            nav.locations_keys.insert(0,'m_board'+str(i))
                elif len(boards) == 1: # 只拍一张 标记位置为board3
                    nav.has_reached_G = True
                    nav.locations['m_board3'] = boards[0].aim_pose
                    nav.locations_keys.insert(0,'m_board3')
                elif len(boards) == 3:
                    nav.has_reached_G = True
                    for i,board in enumerate(boards): # 拍三张
                        nav.locations['m_board'+str(i)] = board.aim_pose
                        nav.locations_keys.insert(0,'m_board'+str(i))
        else: # 被占了 切换模式
            rospy.set_param('mode', 0)
            boards = boardnav.findboard()
            if len(boards) == 2:    
                nav.has_reached_G = True
                if boards[0].uni_shot: # 只拍一张 标记位置为board0/1
                    nav.locations['m_board0'] = boards[0].aim_pose
                    nav.locations_keys.insert(0,'m_board0')
                elif boards[1].uni_shot:
                    nav.locations['m_board1'] = boards[1].aim_pose
                    nav.locations_keys.insert(0,'m_board1')
                else:
                    for i,board in enumerate(boards): # 拍两张
                        nav.locations['m_board'+str(i)] = board.aim_pose
                        nav.locations_keys.insert(0,'m_board'+str(1 - i))
            elif len(boards) == 1: # 只拍一张 标记位置为board3
                nav.has_reached_G = True
                nav.locations['m_board3'] = boards[0].aim_pose
                nav.locations_keys.insert(0,'m_board3')
            elif len(boards) == 3:
                nav.has_reached_G = True
                for i,board in enumerate(boards): # 拍三张
                    nav.locations['m_board'+str(i)] = board.aim_pose
                    nav.locations_keys.insert(0,'m_board'+str(2 - i))

    if nav.location == 'G2':
        boards = get_boards()
        if len(boards) == 2:
            nav.has_reached_G = True
            if boards[0].uni_shot: # 只拍一张 标记位置为board0/1
                nav.locations['m_board0'] = boards[0].aim_pose
                nav.locations_keys.insert(0,'m_board0')
            elif boards[1].uni_shot:
                nav.locations['m_board1'] = boards[1].aim_pose
                nav.locations_keys.insert(0,'m_board1')
            else:
                for i,board in enumerate(boards): # 拍两张
                    nav.locations['m_board'+str(i)] = board.aim_pose
                    nav.locations_keys.insert(0,'m_board'+str(1 - i))
        elif len(boards) == 1: # 只拍一张 标记位置为board3
            nav.has_reached_G = True
            nav.locations['m_board3'] = boards[0].aim_pose
            nav.locations_keys.insert(0,'m_board3')
        elif len(boards) == 3:
            nav.has_reached_G = True
            for i,board in enumerate(boards): # 拍三张
                nav.locations['m_board'+str(i)] = board.aim_pose
                nav.locations_keys.insert(0,'m_board'+str(2 - i))

    if nav.location == 'm_center':
        if nav.mode == 2:
            position = rospy.wait_for_message('/cxy_base_link', Pose, timeout = 5)
            now_angle = euler_from_quaternion([position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w])[2]
            now_angle = np.rad2deg(now_angle)
            angle_list = [-70, -110, 110]
            shot_angle_list = []
            for angle in angle_list:
                shot_angle_list.append(ShotAngle(angle, SHOT_EDGE))
            for board in nav.boards_1:
                tmp_ori = board.aim_pose.orientation
                shot_angle_list.append(ShotAngle(np.rad2deg(euler_from_quaternion([tmp_ori.x, tmp_ori.y, tmp_ori.z, tmp_ori.w])[2]), SHOT_IN))
            turn_list = get_turn_list(now_angle, shot_angle_list)
            # 旋转拍照
            rospy.set_param("can_shot", SHOT_EDGE)
            rospy.loginfo(f">>shot with type {SHOT_EDGE}")
            rospy.sleep(0.5)
            for Angle in turn_list:
                nav.rotate(-1, Angle.angle)
                rospy.set_param("can_shot", Angle.shot_type)
                rospy.loginfo(f">>shot with type {Angle.shot_type}")
                rospy.sleep(0.5)
            # pid F->A 过桥
            rospy.set_param('nongoal_flag', 2)
            rospy.set_param("has_reached_F",2)
            nav.stop_cancel_fail = 0
        if nav.mode == 1:
            rotate_shot_strategy(nav)

    if nav.location.startswith('m_board'):
        rospy.set_param("can_shot", 2) # 内为2

    if nav.location == 'm_right_down':
        rospy.set_param("can_shot", 1)
        rospy.loginfo(">>> shot right ")
        rospy.sleep(0.5)

        nav.rotate(1, 120)
        rospy.set_param("can_shot", 1)
        rospy.loginfo(">>> shot right ")
        rospy.sleep(0.5)

    if nav.location == 'm_left_down':
        #左下拍
        rospy.set_param("can_shot", 1)
        rospy.loginfo(">>> shot left ")
        rospy.sleep(0.5)
        nav.rotate(-1, 70)

        #左上拍
        rospy.set_param("can_shot", 1)
        rospy.loginfo(">>> shot left ")
        rospy.sleep(0.5)
        nav.rotate(1, 35)

        # pid F->A 过桥
        rospy.set_param('nongoal_flag', 2)
        rospy.set_param("has_reached_F",2)
        nav.stop_cancel_fail = 0



def rotate_shot_strategy(nav):
    """
        旋转拍照策略 + 控制F到A pid过桥时机
    """
    sleep_time = 0.5
    rospy.set_param("can_shot", 1)
    rospy.loginfo(">>> shot 0 ")
    rospy.sleep(sleep_time)
    nav.rotate(-1,90)
    rospy.set_param("can_shot", 1)
    rospy.loginfo(">>> shot 1 ")
    rospy.sleep(sleep_time)
    nav.rotate(-1, 90)
    rospy.set_param("can_shot", 1)
    rospy.loginfo(">>> shot 2 ")
    rospy.sleep(sleep_time)
    nav.rotate(-1, 90)
    rospy.set_param("can_shot", 1)
    rospy.loginfo(">>> shot 3 ")
    rospy.sleep(sleep_time)
    # rospy.set_param("can_shot", 1)
    # rospy.loginfo(">>> shot 4 ")
    # rospy.sleep(sleep_time)
    pidF2A(nav)

def pidF2A(nav):
    # pid F->A 过桥
    rospy.set_param('nongoal_flag', 2)
    rospy.set_param("has_reached_F",2)
    nav.stop_cancel_fail = 0

def pidA2F(nav):
    """控制A到F pid 过桥的开/关"""
    #  第一次 A->F 时开启
    if nav.location == a_bridge_first:
        rospy.set_param('nongoal_flag', 3)
        # 将导航主函数阻塞在这里，等待nongoal中pid返还控制权
        while rospy.get_param('nongoal_flag')!= 0:
            rospy.sleep(0.05)  # 20HZ  

