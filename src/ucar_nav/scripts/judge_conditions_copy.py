#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# 问题：
# 遍历出现问题，找到第一个点后不找了
# 老是跳过mboard_1
import rospy
from Utils import *
from findboard import BoardNav
from nav_msgs.msg import OccupancyGrid
import cv2
import dynamic_reconfigure.client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from tf.transformations import euler_from_quaternion
from pid_new import pidControlerWRotate

from findboard import  FindBoard_M2
SHOT_EDGE = 1
SHOT_IN = 2

client_lc_inf = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer") 
client_lc = dynamic_reconfigure.client.Client("/move_base/local_costmap") 
client_gc = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer")
client_teb = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
client_gc_inf = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer") 
# client_gc_ob = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer") 
# client_lc_ob = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer") 


board_target = []

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
def euclidean_distance(vector1, vector2):
    return np.linalg.norm(np.array(vector1) - np.array(vector2))
def angle_between_vectors_with_direction(a, b):
    dot_product = np.dot(a, b)
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    cos_theta = dot_product / (norm_a * norm_b)
    theta_rad = np.arccos(cos_theta)
    theta_deg = np.degrees(theta_rad)
    
    # 使用叉乘判断第二个向量相对于第一个向量的位置
    cross_product = np.cross(a, b)
    direction = np.sign(cross_product)
    theta_deg = theta_deg*direction
    return theta_deg
def angle_between_vectors(a, b):
    dot_product = np.dot(a, b)
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    cos_theta = dot_product / (norm_a * norm_b)
    theta_rad = np.arccos(cos_theta)
    theta_deg = np.degrees(theta_rad)
    return theta_deg
# 0512
def PartFindMethod(car_angel_now,dir_pic2car):        
    # if car_angel_now <= 45 and car_angel_now > -45:
    #     angel_target_boundary = 0
    # elif car_angel_now <= 135 and car_angel_now > 45:
    #     angel_target_boundary = 90
    # elif car_angel_now <= -45 and car_angel_now > -135:
    #     angel_target_boundary = -90
    # elif car_angel_now <= -135 and car_angel_now > -180:
    #     angel_target_boundary = -180
    # elif car_angel_now <= 180 and car_angel_now > 135:
    #     angel_target_boundary = 180
    # else:
    #     angel_target_boundary = car_angel_now
    if car_angel_now <= 30 and car_angel_now > -30:
        angel_target_boundary = 0
    elif car_angel_now <= 120 and car_angel_now > 60:
        angel_target_boundary = 90
    elif car_angel_now <= -60 and car_angel_now > -120:
        angel_target_boundary = -90
    elif car_angel_now <= -150 and car_angel_now > -180:
        angel_target_boundary = -180
    elif car_angel_now <= 180 and car_angel_now > 150:
        angel_target_boundary = 180
    elif car_angel_now <= 60 and car_angel_now > 30:
        if(dir_pic2car==1):
            angel_target_boundary = 90
        else:
            angel_target_boundary = 0
            
    elif car_angel_now <= 150 and car_angel_now > 120:
        if(dir_pic2car==1):
            angel_target_boundary = 180
        else:
            angel_target_boundary = 90
    elif car_angel_now <= -30 and car_angel_now > -60:
        if(dir_pic2car==1):
            angel_target_boundary = 0
        else:
            angel_target_boundary = -90
    elif car_angel_now <= -120 and car_angel_now > -150:
        if(dir_pic2car==1):
            angel_target_boundary = -90
        else:
            angel_target_boundary = -180
    else:
        
        angel_target_boundary = car_angel_now
    return angel_target_boundary
# 0512
def PartFindMethod_board(to_mid): 
    # 定义搜索边界角
    angel_target_boundary = [0,0]
    ## 方法1
    # @Note:区域分区(42,-22,0,22,42)
    margin = 12 # 裕度
    if  to_mid<-0.26:
        angel_target_boundary = [-45-margin,-22+margin]#-80是给了一个裕度
    elif to_mid>=-0.26 and to_mid<0:
        angel_target_boundary = [-22-margin,0+margin]
    elif to_mid>=0 and to_mid<0.26:
        angel_target_boundary = [0-margin,22+margin]
    elif to_mid>=0.26:
        angel_target_boundary = [22-margin,45+margin]
    else:
        angel_target_boundary = [-65,65]
    ## 方法2
    # @Note:to_mid左右裕度(42,-22,0,22,42)
    # if to_mid<0.2 and to_mid>-0.2:
    #     margin = 0.07
    # elif to_mid<-0.4 and to_mid>0.4:
    #     margin = 0.12
    # else:
    #     margin = 0.1
    # angel_target_boundary[0] = np.arctan((to_mid-margin)/(0.5*np.tan(30*3.1415926/180)))
    # angel_target_boundary[1] = np.arctan((to_mid+margin)/(0.5*np.tan(30*3.1415926/180)))
    # angel_target_boundary = np.degrees(angel_target_boundary)
    angel_target_boundary = [-angel_target_boundary[1],-angel_target_boundary[0]]
    print(f"to_mid={to_mid}      搜索边缘角为{angel_target_boundary}")
    print()
    return angel_target_boundary

def start_condition_judge_5(nav):
    """适用于5点和旋转"""
    if nav.location == a_bridge_first:
        rospy.set_param("has_reached_F",1)
        nav.detact_go_out_flag = 0
        # client_teb.update_configuration({'xy_goal_tolerance' : 0.05})
        # client_teb.update_configuration({'yaw_goal_tolerance' : 0.2})
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
def find_dir(msg,boardd):
    global board_target
    boards = boardd
    board_target = boards[0]
    flagx=0
    i=0
    i_target = -1
    dir_pic2car=0       # 识别得到的板子在车的左右
    if  rospy.get_param('rescue_x',  0)<0:
        dir_pic2car=1   # 识别得到的板子在车的左
        board_target.ang_pic2car = 90
    else:
        dir_pic2car=-1  # 识别得到的板子在车的右
        board_target.ang_pic2car = -90
    angel_rela = 0 # 板子与车朝向相对角
    angel_rela_min = 45 # 板子与车朝向相对角最小角度
    angle_edge = 35 # 筛选板子的边缘角
    
    print(f">>一共找到{len(boards)}个板子\n")
    print(f">>识别到板子在车的{dir_pic2car}方向,1为左,-1为右\n")
    angel_car = np.rad2deg(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))[2]
    angel_car_rad = (euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))[2]
    print(f">>车的朝向为：{angel_car}")
    print(f">>车的位置为：{[msg.position.x,msg.position.y]}")
    print()
    for board in boards:
        # 0506
        i = i+1
        print()
        print(f">>板子{i}的位置为{board.position},法线方向为{np.rad2deg(np.arctan2(board.dir[1], board.dir[0]))}")
        print(f">>板子{i}与车的相对矢量为{(board.position- [msg.position.x,msg.position.y])}")
        board.ang_pic2car = angle_between_vectors_with_direction(np.array([np.cos(angel_car_rad), np.sin(angel_car_rad)]),(board.position- [msg.position.x,msg.position.y]))
        print(f">>>>计算得到板子在车的{board.ang_pic2car}方向")
        if(board.ang_pic2car*dir_pic2car<angle_edge and board.ang_pic2car*dir_pic2car>-20):
            # 只识别在车同一侧的板子
            print(f">>>>----板子{i}满足同一侧条件")
            tmp_angle = np.arctan2(board.dir[1], board.dir[0])
            angel_rela = abs(180 + np.rad2deg(tmp_angle) -angel_car) % 360
            if(angel_rela>180):
                angel_rela = 360-angel_rela
            print(f">>>>板子{i}法线方向{np.rad2deg(tmp_angle)}与车的方向夹角为{angel_rela}")
            print()
            if angel_rela< angel_rela_min+8:
                if (angel_rela > angel_rela_min - 10 and board.ang_pic2car*dir_pic2car<board_target.ang_pic2car*dir_pic2car) or (angel_rela < angel_rela_min - 10):
                    angel_rela_min = angel_rela
                    i_target = i
                    flagx=1
                    board_target = board
    if(flagx==1):
        # 说明找到了目标板
        if  angel_rela_min<20 and (board_target.aim_pose.position.x-msg.position.x)*(board_target.aim_pose.position.x-msg.position.x)+(board_target.aim_pose.position.y-msg.position.y)*(board_target.aim_pose.position.y-msg.position.y)<0.5:
            rospy.set_param('flag_PID2', 1)
        else:
            rospy.set_param('flag_PID2', 0)
        print(f"目标板为第{i_target}个板子，朝向为:{np.rad2deg(np.arctan2(board_target.dir[1], board_target.dir[0]))}")
    else:
        print("未找到板子,直接进入PID")
        angel_target_boundary = PartFindMethod(angel_car,dir_pic2car)
        angel_car1=float(angel_car)
        rospy.set_param('angel_target_boundary_rad', 3.1415/180*angel_target_boundary)
        rospy.set_param('angel_car2boundary', angel_target_boundary-angel_car1)
        print()
        print(angel_target_boundary) 
        print(f"车头需要转弯{rospy.get_param('angel_car2boundary', 0)}度到达{angel_target_boundary}") 
        
    rospy.set_param('flag_findornot_nav', flagx)    #未找到板子0;找到了目标板1
    return flagx

def find_dir_v2(msg,boards):
    global board_target
    rospy.set_param('get_board', 0)
    board_target = []
    ## 1.初始化参数
    flagx=0 # 导航匹配标志位
    i=0 # 迭代量
    i_target = [] # 目标板位置
    dir_pic2car=0       # 识别得到的板子在车的左右
    if  rospy.get_param('rescue_x',  0)<0:
        dir_pic2car=1   # 识别得到的板子在车的左
    else:
        dir_pic2car=-1  # 识别得到的板子在车的右
    distance_rela = 0 # 板子与车朝向相对角
    distance_rela_min = 5 # 板子与车朝向相对角最小角度 
    ## 2.获取搜索区域边界
    part_pic2car_edge = PartFindMethod_board(rospy.get_param('rescue_x',  0))
    print(f">>一共找到{len(boards)}个板子\n")
    print(f">>搜索区域为{part_pic2car_edge}")
    print()
    ## 3.获取车体信息
    angel_car = np.rad2deg(euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))[2]
    angel_car_rad = (euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))[2]
    print(f">>车的朝向为：{angel_car}")
    print(f">>车的位置为：{[msg.position.x,msg.position.y]}")
    print()
    ## 4.遍历所有导航找到的板子，分析是否在搜索区域内
    # @Note 若有，则返回1，并且全部放在board_target里面供接下来遍历用；若没有，则返回0，接下来前往下一个旋转点
    for board in boards:
        i = i+1
        print()
        print(f">>板子{i}的位置为{board.position},法线方向为{np.rad2deg(np.arctan2(board.dir[1], board.dir[0]))}")
        print(f">>板子{i}与车的相对矢量为{(board.position- [msg.position.x,msg.position.y])}")
        board.ang_pic2car = angle_between_vectors_with_direction(np.array([np.cos(angel_car_rad), np.sin(angel_car_rad)]),(board.position- [msg.position.x,msg.position.y]))
        print(f">>>>计算得到板子中心点的位置在车的{board.ang_pic2car}方向")
        if(board.ang_pic2car<part_pic2car_edge[1] and board.ang_pic2car>part_pic2car_edge[0] and (board.position[0]>-2 or board.position[1]>-1.6)):
            # 搜索到板子在区域内，并且不是急救包
            flagx=1
            print(f">>>>----板子{i}在搜索区域,添加到'board_target'")
            board_target.append(board)
            i_target.append(i)
            distance_rela = euclidean_distance(board.position,[msg.position.x,msg.position.y])
            print(f">>>>板子{i}与车的欧式距离为{distance_rela}")
            print()
    if(flagx==1):
        # 说明找到了目标板
        # if (board_target.aim_pose.position.x-msg.position.x)*(board_target.aim_pose.position.x-msg.position.x)+(board_target.aim_pose.position.y-msg.position.y)*(board_target.aim_pose.position.y-msg.position.y)<0.5:
        #     rospy.set_param('flag_PID2', 1)
        # else:
        #     rospy.set_param('flag_PID2', 0)
        print(f"目标板为第{i_target}个板子")
    else:
        flagx = 0
        print("未找到板子!!!")
        # 判断是否需要直接执行PID
        angel_target_boundary = PartFindMethod(angel_car,dir_pic2car)
        if msg.position.y<0.4 and msg.position.y>-0.4:
            print("此时车在'center'")
            # 如果目标板在前或后，则建议直接进入PID
            if(angel_target_boundary == 0 or angel_target_boundary == 180 or angel_target_boundary == -180):
                # print("直接进入PID")
                # angel_car1=float(angel_car)
                # rospy.set_param('angel_target_boundary_rad', 3.1415/180*angel_target_boundary)
                # rospy.set_param('angel_car2boundary', angel_target_boundary-angel_car1)
                # print()
                # print(angel_target_boundary) 
                # print(f"车头需要转弯{rospy.get_param('angel_car2boundary', 0)}度到达{angel_target_boundary}") 
                print("此时车在'center'但看不到板子!使用新方法！")
                new_board = Board()
                new_board.aim_pose = FindBoard_M2()
                board_target.append(new_board)
                flagx = 1
            else:
                flagx = -1
                print("前往下一个旋转点")
        # else:
        #     print("此时车不在'center'!")
        #     print("直接进入PID")
        #     angel_car1=float(angel_car)
        #     rospy.set_param('angel_target_boundary_rad', 3.1415/180*angel_target_boundary)
        #     rospy.set_param('angel_car2boundary', angel_target_boundary-angel_car1)
        #     print()
        #     print(angel_target_boundary) 
        #     print(f"车头需要转弯{rospy.get_param('angel_car2boundary', 0)}度到达{angel_target_boundary}") 
        elif msg.position.y<-0.4:
            print("此时车不在'center'!使用新方法！")
            new_board = Board()
            new_board.aim_pose = FindBoard_M2()
            board_target.append(new_board)
            flagx = 1
        else:
            print("此时车不在'center'!")
            print("直接进入PID")
            angel_car1=float(angel_car)
            rospy.set_param('angel_target_boundary_rad', 3.1415/180*angel_target_boundary)
            rospy.set_param('angel_car2boundary', angel_target_boundary-angel_car1)
    rospy.set_param('flag_findornot_nav', flagx)    #未找到板子0;找到了目标板1
    print(f"flag_findornot_nav是{flagx}        1:找到了目标板,准备前往 | 0:直接进入PID | -1:没有匹配上，前往下一点")
    return flagx



def shot_concise(nav):
    """ 控制开始雷达扫描板子导航"""
    # 抵达G区域内后
    global board_target 
    # if rospy.get_param('findboard', 0) == 1 and rospy.get_param('rescue',  0) != 4: 
    boards = get_boards()
    # print(boards)
    position = rospy.wait_for_message('/cxy_base_link', Pose, timeout = 5)
    if(find_dir_v2(position,boards)!=1):
        zps=250
        # rospy.set_param('findboard', 2)
        # zps = rospy.Subscriber('/cxy_base_link', Pose, find_dir,callback_args=(boards,))
        # if len(boards) == 2:
        #     nav.has_reached_G = True
    # elseif rospy.get_param('flag_findornot_nav', 0) == 0
    # elif rospy.get_param('flag_PID2', 0) == 1 :
    #     rospy.set_param('findboard', 2)
    else:
        # nav.locations['m_board0'] = board_target.aim_pose
        # 
        nav.locations['m_board0'] = board_target[0].aim_pose
        nav.locations_keys.insert(0,'m_board0')
        print(f"board_target[0].aim_pose是:{board_target[0].aim_pose}")
        if(len(board_target)==2):
            nav.locations['m_board1'] = board_target[1].aim_pose
            nav.locations_keys.insert(0,'m_board1')
            print(f"board_target[1].aim_pose是:{board_target[1].aim_pose}")
        rospy.set_param('findboard', 2)
    if rospy.get_param("direction", 0)==2 and rospy.get_param('flag_findornot_nav',  0) == -1 :#(rospy.get_param('flag_findornot_nav',  0) != 1 and rospy.get_param('flag_findornot_nav',  0) != 4)
        rospy.set_param("set_goal", 2)
    elif rospy.get_param("direction", 0)==3 and rospy.get_param('flag_findornot_nav',  0) == -1 :#(rospy.get_param('flag_findornot_nav',  0) != 1 and rospy.get_param('flag_findornot_nav',  0) != 4)
        rospy.set_param("set_goal", 3)    
    
        # rospy.set_param('findboard', 2)
        #     elif boards[1].uni_shot:
        #         nav.locations['m_board1'] = boards[1].aim_pose
        #         nav.locations_keys.insert(0,'m_board1')
        #     else:
        #         for i,board in enumerate(boards): # 拍两张
        #             nav.locations['m_board'+str(i)] = board.aim_pose
        #             nav.locations_keys.insert(0,'m_board'+str(1 - i))
        # elif len(boards) == 1: # 只拍一张 标记位置为board3
        #     nav.has_reached_G = True
        #     nav.locations['m_board3'] = boards[0].aim_pose
        #     nav.locations_keys.insert(0,'m_board3')
        # elif len(boards) == 3:
        #     nav.has_reached_G = True
        #     for i,board in enumerate(boards): # 拍三张
        #         nav.locations['m_board'+str(i)] = board.aim_pose
        #         nav.locations_keys.insert(0,'m_board'+str(2- i))

    # if nav.location == 'G2':
    #     boards = get_boards()
    #     if len(boards) == 2:
    #         nav.has_reached_G = True
    #         if boards[0].uni_shot: # 只拍一张 标记位置为board0/1
    #             nav.locations['m_board0'] = boards[0].aim_pose
    #             nav.locations_keys.insert(0,'m_board0')
    #         elif boards[1].uni_shot:
    #             nav.locations['m_board1'] = boards[1].aim_pose
    #             nav.locations_keys.insert(0,'m_board1')
    #         else:
    #             for i,board in enumerate(boards): # 拍两张
    #                 nav.locations['m_board'+str(i)] = board.aim_pose
    #                 nav.locations_keys.insert(0,'m_board'+str(1 - i))
    #     elif len(boards) == 1: # 只拍一张 标记位置为board3
    #         nav.has_reached_G = True
    #         nav.locations['m_board3'] = boards[0].aim_pose
    #         nav.locations_keys.insert(0,'m_board3')
    #     elif len(boards) == 3:
    #         nav.has_reached_G = True
    #         for i,board in enumerate(boards): # 拍三张
    #             nav.locations['m_board'+str(i)] = board.aim_pose
    #             nav.locations_keys.insert(0,'m_board'+str(2 - i))

    # if nav.location.startswith('m_board'):
    #     rospy.set_param("can_shot", 2) # 内为2

    # if nav.location == 'm_right_down':
    #     rospy.set_param("can_shot", 1)
    #     rospy.loginfo(">>> shot right ")
    #     rospy.sleep(0.5)

    #     nav.rotate(1, 120)
    #     rospy.set_param("can_shot", 1)
    #     rospy.loginfo(">>> shot right ")
    #     rospy.sleep(0.5)

    # if nav.location == 'm_left_down':
    #     #左下拍
    #     rospy.set_param("can_shot", 1)
    #     rospy.loginfo(">>> shot left ")
    #     rospy.sleep(0.5)
    #     nav.rotate(-1, 70)

    #     #左上拍
    #     rospy.set_param("can_shot", 1)
    #     rospy.loginfo(">>> shot left ")
    #     rospy.sleep(0.5)
    #     nav.rotate(1, 35)

    #     # pid F->A 过桥
    #     rospy.set_param('nongoal_flag', 2)
    #     rospy.set_param("has_reached_F",2)
    #     nav.stop_cancel_fail = 0
        

def rotatePic(data1):
    data = []
    for x in data1:
        if x <0:
            data.append(0)
        else:
            data.append(x)

    data = np.array(data)
    data = data.reshape(132, -1)
    image = data.astype(np.uint8)
    height, width = image.shape[:2]
    center = (width/2, height/2)
    rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=6, scale=1)
    rotated_image = cv2.warpAffine(src=image, M=rotate_matrix, dsize=(width, height))
    return image #rotated_image

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
        data = data.reshape(132, -1)

        image = data.astype(np.uint8)
        height, width = image.shape[:2]
        center = (width/2, height/2)
        #rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=1, scale=1)
        #rotated_image = cv2.warpAffine(src=image, M=rotate_matrix, dsize=(width, height))
        return image
    rotated_image = rotatePic(data1)
    # 第一个值控制上下 第二个值控制左右
    leftDown = (27,25) # [0] 变大可以让车更往F房间深处走102，61#28
    sideLength = 40
    cutOutPic = np.zeros((80, 40)).astype(np.uint8)
    for x in range(80):
        for y in range(40):
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
    boards = boardnav.findboard(position) #当找到两个板子时，令has_reached_G为true
    # rospy.loginfo(boards.position)
    # rospy.loginfo(boards.aim_pose)
    return boards

    
    
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

# def combined(nav):
#     if nav.location == 'G1':
#         # boardnav
#         boardnav = BoardNav()
#         rospy.loginfo('>>>shot from laser info')
#         position = rospy.wait_for_message('/cxy_base_link', Pose, timeout = 5)
#         boardnav.now_position = (position.position.x, position.position.y)
#         data = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout = 5)
#         rospy.loginfo((position.position.x, position.position.y))
#         boardnav.resolution = data.info.resolution
#         boardnav.img = gen_cutPic(data.data)
#         boards = boardnav.rotate_strategy()
#         nav.boards_1 = boards
#         occupied = False
#         for board in boards:
#             # 障碍板是否在中央点附近
#             if point_in_convex_polygon(board.position, boardnav.get_central_square().flatten().tolist()):
#                 print('旋转中央点被占据')
#                 occupied = True
#                 break
#         if not occupied: # 没被占
#             print('旋转中央点wei被占据')
#             if nav.mode == 2:
#                 nav.locations_keys.pop(0) # 弹出G2
#                 nav.locations_keys.pop(0) # 弹出m_right_down
#                 nav.locations_keys.pop(0) # 弹出m_left_down
#                 nav.locations['m_center'] = Pose(Point(1.009, -3.395, 0.000), Quaternion(0.000, 0.000, 0.439, 0.898))
#                 nav.locations_keys.insert(0,'m_center')
#             if nav.mode == 1:
#                 nav.locations_keys.pop(0) # 弹出G2
#                 nav.locations_keys.pop(0) # 弹出m_right_down
#                 nav.locations_keys.pop(0) # 弹出m_left_down
#                 # nav.locations['m_center'] = Pose(Point(1.009, -3.395, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
#                 nav.locations['m_center'] = Pose(Point(1.009, -3.395, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
#                 nav.locations_keys.insert(0,'m_center')
#             if nav.mode == 0:
#                 # nav.locations_keys.pop(0) # 弹出G2
#                 boards = boardnav.findboard()
#                 if len(boards) == 2:
#                     nav.has_reached_G = True
#                     if boards[0].uni_shot: # 只拍一张 标记位置为board0/1
#                         nav.locations['m_board0'] = boards[0].aim_pose
#                         nav.locations_keys.insert(0,'m_board0')
#                     elif boards[1].uni_shot:
#                         nav.locations['m_board1'] = boards[1].aim_pose
#                         nav.locations_keys.insert(0,'m_board1')
#                     else:
#                         for i,board in enumerate(boards): # 拍两张
#                             nav.locations['m_board'+str(i)] = board.aim_pose
#                             nav.locations_keys.insert(0,'m_board'+str(i))
#                 elif len(boards) == 1: # 只拍一张 标记位置为board3
#                     nav.has_reached_G = True
#                     nav.locations['m_board3'] = boards[0].aim_pose
#                     nav.locations_keys.insert(0,'m_board3')
#                 elif len(boards) == 3:
#                     nav.has_reached_G = True
#                     for i,board in enumerate(boards): # 拍三张
#                         nav.locations['m_board'+str(i)] = board.aim_pose
#                         nav.locations_keys.insert(0,'m_board'+str(i))
#         else: # 被占了 切换模式
#             rospy.set_param('mode', 0)
#             boards = boardnav.findboard()
#             if len(boards) == 2:    
#                 nav.has_reached_G = True
#                 if boards[0].uni_shot: # 只拍一张 标记位置为board0/1
#                     nav.locations['m_board0'] = boards[0].aim_pose
#                     nav.locations_keys.insert(0,'m_board0')
#                 elif boards[1].uni_shot:
#                     nav.locations['m_board1'] = boards[1].aim_pose
#                     nav.locations_keys.insert(0,'m_board1')
#                 else:
#                     for i,board in enumerate(boards): # 拍两张
#                         nav.locations['m_board'+str(i)] = board.aim_pose
#                         nav.locations_keys.insert(0,'m_board'+str(1 - i))
#             elif len(boards) == 1: # 只拍一张 标记位置为board3
#                 nav.has_reached_G = True
#                 nav.locations['m_board3'] = boards[0].aim_pose
#                 nav.locations_keys.insert(0,'m_board3')
#             elif len(boards) == 3:
#                 nav.has_reached_G = True
#                 for i,board in enumerate(boards): # 拍三张
#                     nav.locations['m_board'+str(i)] = board.aim_pose
#                     nav.locations_keys.insert(0,'m_board'+str(2 - i))

#     if nav.location == 'G2':
#         boards = get_boards()
#         if len(boards) == 2:
#             nav.has_reached_G = True
#             if boards[0].uni_shot: # 只拍一张 标记位置为board0/1
#                 nav.locations['m_board0'] = boards[0].aim_pose
#                 nav.locations_keys.insert(0,'m_board0')
#             elif boards[1].uni_shot:
#                 nav.locations['m_board1'] = boards[1].aim_pose
#                 nav.locations_keys.insert(0,'m_board1')
#             else:
#                 for i,board in enumerate(boards): # 拍两张
#                     nav.locations['m_board'+str(i)] = board.aim_pose
#                     nav.locations_keys.insert(0,'m_board'+str(1 - i))
#         elif len(boards) == 1: # 只拍一张 标记位置为board3
#             nav.has_reached_G = True
#             nav.locations['m_board3'] = boards[0].aim_pose
#             nav.locations_keys.insert(0,'m_board3')
#         elif len(boards) == 3:
#             nav.has_reached_G = True
#             for i,board in enumerate(boards): # 拍三张
#                 nav.locations['m_board'+str(i)] = board.aim_pose
#                 nav.locations_keys.insert(0,'m_board'+str(2 - i))

#     if nav.location == 'm_center':
#         if nav.mode == 2:
#             position = rospy.wait_for_message('/cxy_base_link', Pose, timeout = 5)
#             now_angle = euler_from_quaternion([position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w])[2]
#             now_angle = np.rad2deg(now_angle)
#             angle_list = [-70, -110, 110]
#             shot_angle_list = []
#             for angle in angle_list:
#                 shot_angle_list.append(ShotAngle(angle, SHOT_EDGE))
#             for board in nav.boards_1:
#                 tmp_ori = board.aim_pose.orientation
#                 shot_angle_list.append(ShotAngle(np.rad2deg(euler_from_quaternion([tmp_ori.x, tmp_ori.y, tmp_ori.z, tmp_ori.w])[2]), SHOT_IN))
#             turn_list = get_turn_list(now_angle, shot_angle_list)
#             # 旋转拍照
#             rospy.set_param("can_shot", SHOT_EDGE)
#             rospy.loginfo(f">>shot with type {SHOT_EDGE}")
#             rospy.sleep(0.5)
#             for Angle in turn_list:
#                 nav.rotate(-1, Angle.angle)
#                 rospy.set_param("can_shot", Angle.shot_type)
#                 rospy.loginfo(f">>shot with type {Angle.shot_type}")
#                 rospy.sleep(0.5)
#             # pid F->A 过桥
#             rospy.set_param('nongoal_flag', 2)
#             rospy.set_param("has_reached_F",2)
#             nav.stop_cancel_fail = 0
#         if nav.mode == 1:
#             rotate_shot_strategy(nav)

#     if nav.location.startswith('m_board'):
#         rospy.set_param("can_shot", 2) # 内为2

#     if nav.location == 'm_right_down':
#         rospy.set_param("can_shot", 1)
#         rospy.loginfo(">>> shot right ")
#         rospy.sleep(0.5)

#         nav.rotate(1, 120)
#         rospy.set_param("can_shot", 1)
#         rospy.loginfo(">>> shot right ")
#         rospy.sleep(0.5)

#     if nav.location == 'm_left_down':
#         #左下拍
#         rospy.set_param("can_shot", 1)
#         rospy.loginfo(">>> shot left ")
#         rospy.sleep(0.5)
#         nav.rotate(-1, 70)

#         #左上拍
#         rospy.set_param("can_shot", 1)
#         rospy.loginfo(">>> shot left ")
#         rospy.sleep(0.5)
#         nav.rotate(1, 35)

#         # pid F->A 过桥
#         rospy.set_param('nongoal_flag', 2)
#         rospy.set_param("has_reached_F",2)
#         nav.stop_cancel_fail = 0



# def rotate_shot_strategy(nav):
#     """
#         旋转拍照策略
#     """
#     # rospy.set_param("flag_findornot_nav", 4)
#     if(str(nav.location)=='rotate1'  or str(nav.location)=='rotate2' or str(nav.location)=='rotate10' ):            #单点备用顺时针一圈
#         rospy.set_param("take_photo", 1)
#         sleep_time = 0.4
#         rospy.sleep(sleep_time)
#         for i in range(5):
#             if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#                 rospy.set_param('findboard', 1)
#                 rospy.loginfo(rospy.get_param('rescue',  0))
#                 break
#             elif rospy.get_param('rescue',  0) == 4:
#                 nav.move_base.cancel_goal()
#                 break
#             else:
#                 rospy.loginfo(">>> shot  ")
#                 # nav.rotate(-1,43)#7
#                 nav.rotate(-1,45)
#                 rospy.sleep(sleep_time)
#         nav.move_base.cancel_goal()
#      ############################################################################################
#     if str(nav.location)=='center' :                                #下桥试探
#         rospy.set_param("take_photo", 1)
#         sleep_time = 0.5

#         rospy.set_param("direction", 1)#中间
#         # rospy.sleep(sleep_time)
#         # if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品并执行找板
#         #         rospy.set_param('findboard', 1)
#         #         rospy.loginfo(rospy.get_param('rescue',  0))
#         # elif rospy.get_param('rescue',  0) == 4:
#         #         nav.move_base.cancel_goal()
#         # else:
#         #         rospy.loginfo(">>> shot  ")
#         if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#             rospy.set_param('findboard', 1)
#             rospy.loginfo(rospy.get_param('rescue',  0))
#             # nav.move_base.cancel_goal()
#             return 
#         elif rospy.get_param('rescue',  0) == 4:
#             nav.move_base.cancel_goal()
#             return
#         else:
#             rospy.loginfo(">>> shot  ")
#         if rospy.get_param('set_goal',  0) != 0:
#             nav.move_base.cancel_goal()
#             return
#             # nav.rotate(-1,43)#7
#             # nav.rotate(-1,12)
#             rospy.sleep(0.5)

#         nav.rotate(-1,20)
#         rospy.set_param("direction", 2)#右侧
#         if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#             rospy.set_param('findboard', 1)
#             rospy.loginfo(rospy.get_param('rescue',  0))
#             nav.move_base.cancel_goal()
#             return 
#         elif rospy.get_param('rescue',  0) == 4:
#             nav.move_base.cancel_goal()
#             return
#         else:
#             rospy.loginfo(">>> shot  ")
#         if rospy.get_param('set_goal',  0) != 0:
#             nav.move_base.cancel_goal()
#             return
#             # nav.rotate(-1,43)#7
#             # nav.rotate(-1,12)
#             rospy.sleep(0.5)
#         nav.rotate(-1,25)
        
#         for i in range(3):
#             if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#                 rospy.set_param('findboard', 1)
#                 rospy.loginfo(rospy.get_param('rescue',  0))
#                 nav.move_base.cancel_goal()
#                 return
#             elif rospy.get_param('rescue',  0) == 4:
#                 nav.move_base.cancel_goal()
#                 return
#             else:
#                 rospy.loginfo(">>> shot  ")
#                 # nav.rotate(-1,43)#7
#                 nav.rotate(-1,15)
#                 rospy.sleep(0.5)
#             if rospy.get_param('set_goal',  0) != 0:
#                 nav.move_base.cancel_goal()
#                 return

#         nav.rotate(-1,45)
#         rospy.set_param("direction", 3)#右侧
#         nav.rotate(-1,45)

#         for i in range(5):
#             if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#                 rospy.set_param('findboard', 1)
#                 rospy.loginfo(rospy.get_param('rescue',  0))
#                 nav.move_base.cancel_goal()
#                 return
#             elif rospy.get_param('rescue',  0) == 4:
#                 nav.move_base.cancel_goal()
#                 return
#             else:
#                 rospy.loginfo(">>> shot  ")
#                 # nav.rotate(-1,43)#7
#                 nav.rotate(-1,30)
#                 rospy.sleep(0.5)
#             if rospy.get_param('set_goal',  0) != 0:
#                 nav.move_base.cancel_goal()
#                 return
#         # if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#         #     rospy.set_param('findboard', 1)
#         #     rospy.loginfo(rospy.get_param('rescue',  0))
#         #     nav.move_base.cancel_goal()
#         #     return 
#         # elif rospy.get_param('rescue',  0) == 4:
#         #     nav.move_base.cancel_goal()
#         #     return
#         # else:
#         #     rospy.loginfo(">>> shot  ")
#         # rospy.sleep(sleep_time)

#         # nav.rotate(-1,45)
#         # if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#         #     rospy.set_param('findboard', 1)
#         #     rospy.loginfo(rospy.get_param('rescue',  0))
#         #     nav.move_base.cancel_goal()
#         #     return 
#         # elif rospy.get_param('rescue',  0) == 4:
#         #     nav.move_base.cancel_goal()
#         #     return
#         # else:
#         #     rospy.loginfo(">>> shot  ")

#         # rospy.sleep(sleep_time)

#         # nav.rotate(-1,45)
#         # if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#         #     rospy.set_param('findboard', 1)
#         #     rospy.loginfo(rospy.get_param('rescue',  0))
#         #     nav.move_base.cancel_goal()
#         #     return 
#         # elif rospy.get_param('rescue',  0) == 4:
#         #     nav.move_base.cancel_goal()
#         #     return
#         # else:
#         #     rospy.loginfo(">>> shot  ")
#         # rospy.sleep(sleep_time)#sleep_time+1
#         # # if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品并执行找板
#         # #         rospy.set_param('findboard', 1)
#         # #         rospy.loginfo(rospy.get_param('rescue',  0))
#         # # elif rospy.get_param('rescue',  0) == 4:
#         # #         nav.move_base.cancel_goal()
#         # # else:
#         # #         rospy.loginfo(">>> shot  ")

#         nav.move_base.cancel_goal()
# ################################################################################################3
#     if str(nav.location)=='rotate20':                                                                              #单点备用逆时针一圈
#         rospy.set_param("take_photo", 1)
#         sleep_time = 1
#         rospy.sleep(sleep_time)
#         for i in range(4):
#             if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#                 rospy.set_param('findboard', 1)
#                 rospy.loginfo(rospy.get_param('rescue',  0))
#                 break
#             elif rospy.get_param('rescue',  0) == 4:
#                 nav.move_base.cancel_goal()
#                 break
#             else:
#                 rospy.loginfo(">>> shot  ")
#                 # nav.rotate(-1,43)#7
#                 nav.rotate(-1,60)
#                 rospy.sleep(sleep_time)
#         nav.move_base.cancel_goal()   

#     if(str(nav.location)=='rotate111'  or str(nav.location)=='rotate22' ):            #顺时针半圈
#         rospy.set_param("take_photo", 1)
#         sleep_time = 1
#         rospy.sleep(sleep_time)
#         for i in range(4):
#             if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#                 rospy.set_param('findboard', 1)
#                 rospy.loginfo(rospy.get_param('rescue',  0))
#                 break
#             elif rospy.get_param('rescue',  0) == 4:
#                 nav.move_base.cancel_goal()
#                 break
#             else:
#                 rospy.loginfo(">>> shot  ")
#                 # nav.rotate(-1,43)#7
#                 nav.rotate(-1,60)
#                 rospy.sleep(sleep_time)
#         nav.move_base.cancel_goal()

#     if(str(nav.location)=='rotate11'  or str(nav.location)=='rotate222' ):            #逆时针半圈
#         rospy.set_param("take_photo", 1)
#         sleep_time = 1
#         rospy.sleep(sleep_time)
#         for i in range(4):
#             if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品  
#                 rospy.set_param('findboard', 1)
#                 rospy.loginfo(rospy.get_param('rescue',  0))
#                 break
#             elif rospy.get_param('rescue',  0) == 4:
#                 nav.move_base.cancel_goal()
#                 break
#             else:
#                 rospy.loginfo(">>> shot  ")
#                 # nav.rotate(-1,43)#7
#                 nav.rotate(1,60)
#                 rospy.sleep(sleep_time)
#         nav.move_base.cancel_goal()
def rotate_shot_strategy(nav):
    """
        旋转拍照策略
    """
    if(str(nav.location)=='rotate1'  or str(nav.location)=='rotate2' or str(nav.location)=='rotate20'):            #单点备用顺时针一圈
        rospy.set_param("take_photo", 1)
        sleep_time = 0.4
        rospy.sleep(sleep_time)
        for i in range(12):
            rospy.sleep(sleep_time)
            print(f"第{i}个旋转点rescue是{rospy.get_param('rescue',  0)},findboard是{rospy.get_param('findboard', 0)}")
            print()
            if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 3 and rospy.set_param("rescue", 0)!=4:             #找到目标物品  
                rospy.set_param('findboard', 1)
                rospy.loginfo(rospy.get_param('rescue',  0))
                break
            elif rospy.get_param('rescue',  0) == 4:
                break
            else:
                rospy.loginfo(">>> shot  ")
                # nav.rotate(-1,43)#7
                
                nav.rotate(-1,30)
                
        nav.move_base.cancel_goal()

    if str(nav.location)=='center' :                                #下桥试探
        print("到达center!!!")
        rospy.set_param("take_photo", 1)
        sleep_time = 0.4
        rospy.sleep(1)
        rospy.set_param("direction", 1)#中间
        # rospy.sleep(sleep_time)
        # if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 2:             #找到目标物品并执行找板
        #         rospy.set_param('findboard', 1)
        #         rospy.loginfo(rospy.get_param('rescue',  0))
        # elif rospy.get_param('rescue',  0) == 4:
        #         nav.move_base.cancel_goal()
        # else:
        #         rospy.loginfo(">>> shot  ")
        rospy.sleep(sleep_time)
        if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 3:             #找到目标物品  
            rospy.set_param('findboard', 1)
            rospy.loginfo(rospy.get_param('rescue',  0))
            nav.move_base.cancel_goal()
            return 
        elif rospy.get_param('rescue',  0) == 4:
            nav.move_base.cancel_goal()
            return
        else:
            rospy.loginfo(">>> shot  ")
            # nav.rotate(-1,43)#7
            # nav.rotate(-1,12)
            

        nav.rotate(-1,20)
        rospy.sleep(sleep_time)
        if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 3:             #找到目标物品  
            rospy.set_param('findboard', 1)
            rospy.loginfo(rospy.get_param('rescue',  0))
            nav.move_base.cancel_goal()
            return 
        elif rospy.get_param('rescue',  0) == 4:
            nav.move_base.cancel_goal()
            return
        else:
            rospy.loginfo(">>> shot  ")
            # nav.rotate(-1,43)#7
            # nav.rotate(-1,12)
            
        nav.rotate(-1,25)
        rospy.set_param("direction", 2)#右侧
    

        for i in range(6):
            rospy.sleep(sleep_time)
            print(f"第{i}个旋转点rescue是{rospy.get_param('rescue',  0)},findboard是{rospy.get_param('findboard', 0)}")
            if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 3:             #找到目标物品  
                rospy.set_param('findboard', 1)
                rospy.loginfo(rospy.get_param('rescue',  0))
                nav.move_base.cancel_goal()
                return
            elif rospy.get_param('rescue',  0) == 4:
                nav.move_base.cancel_goal()
                return
            else:
                rospy.loginfo(">>> shot  ")
                # nav.rotate(-1,43)#7
                nav.rotate(-1,15)
                

        nav.rotate(-1,45)
        rospy.set_param("direction", 3)#右侧
        nav.rotate(-1,15)

        for i in range(6):
            rospy.sleep(sleep_time)
            print(f"第{i}个旋转点rescue是{rospy.get_param('rescue',  0)},findboard是{rospy.get_param('findboard', 0)}")
            if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 3:             #找到目标物品  
                rospy.set_param('findboard', 1)
                rospy.loginfo(rospy.get_param('rescue',  0))
                nav.move_base.cancel_goal()
                return
            elif rospy.get_param('rescue',  0) == 4:
                nav.move_base.cancel_goal()
                return
            else:
                rospy.loginfo(">>> shot  ")
                # nav.rotate(-1,43)#7
                nav.rotate(-1,15)
                

        nav.move_base.cancel_goal()
    else:
        rospy.set_param("direction", 0)#恢复

    # if str(nav.location)=='rotate11':                                                                              #单点备用逆时针一圈
    #     rospy.set_param("take_photo", 1)
    #     sleep_time = 0.5
    #     rospy.sleep(sleep_time)
    #     for i in range(6):
    #         if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 3 and rospy.set_param("rescue", 0)!=4:             #找到目标物品  
    #             rospy.set_param('findboard', 1)
    #             rospy.loginfo(rospy.get_param('rescue',  0))
    #             break
    #         elif rospy.get_param('rescue',  0) == 4:
    #             nav.move_base.cancel_goal()
    #             break
    #         else:
    #             rospy.loginfo(">>> shot  ")
    #             # nav.rotate(-1,43)#7
    #             nav.rotate(1,30)
    #             rospy.sleep(sleep_time)
    #     nav.move_base.cancel_goal()   
        


    if(str(nav.location)=='rotate111'  or str(nav.location)=='rotate22' or str(nav.location)=='rotate10' ):            #顺时针半圈
        rospy.set_param("take_photo", 1)
        sleep_time = 0.4
        rospy.sleep(sleep_time)
        for i in range(12):
            rospy.sleep(sleep_time)
            print(f"第{i}个旋转点rescue是{rospy.get_param('rescue',  0)},findboard是{rospy.get_param('findboard', 0)}")
            if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 3 and rospy.set_param("rescue", 0)!=4:             #找到目标物品  
                rospy.set_param('findboard', 1)
                rospy.loginfo(rospy.get_param('rescue',  0))
                break
            elif rospy.get_param('rescue',  0) == 4:
                nav.move_base.cancel_goal()
                break
            else:
                rospy.loginfo(">>> shot  ")
                # nav.rotate(-1,43)#7
                nav.rotate(-1,30)
                
        nav.move_base.cancel_goal()

    if(str(nav.location)=='rotate11'  or str(nav.location)=='rotate222' ):            #逆时针半圈
        rospy.set_param("take_photo", 1)
        sleep_time = 0.4
        rospy.sleep(sleep_time)
        for i in range(6):
            rospy.sleep(sleep_time)
            if rospy.get_param('rescue',  0) == 3 and rospy.get_param('findboard', 0) != 3 and rospy.set_param("rescue", 0)!=4:             #找到目标物品  
                rospy.set_param('findboard', 1)
                rospy.loginfo(rospy.get_param('rescue',  0))
                break
            elif rospy.get_param('rescue',  0) == 4:
                nav.move_base.cancel_goal()
                break
            else:
                rospy.loginfo(">>> shot  ")
                # nav.rotate(-1,43)#7
                nav.rotate(1,30)
                
        nav.move_base.cancel_goal()



def insertnew(nav):                                                                                                                                             #插入备用点
    # # F内3*3区域的取点 不妨认为其为房间G
    # if str(nav.location)=='rotate1' :
    #     #三点备用
    #     nav.locations['rotate10'] = Pose(Point(-1.83,-1.65, 0.000), Quaternion(0.000, 0.000, 0.346,0.94))
    #     nav.locations_keys.insert(0,'rotate10')
    # if str(nav.location)=='rotate2' :
    #     #三点备用
    #     nav.locations['rotate20'] = Pose(Point(-1.827,1.33, 0.000), Quaternion(0.000, 0.000, -0.95,0.315))
    #     nav.locations_keys.insert(0,'rotate20')
    if str(nav.location)=='rotate1' :
        nav.locations['rotate111'] = Pose(Point(-1.327,-1.45, 0.000), Quaternion(0.000, 0.000, 0.48,0.88)) 
        nav.locations_keys.insert(0,'rotate111')
        nav.locations['rotate11'] = Pose(Point(-2.55, -1.83, 0.000),Quaternion(0.000, 0.000, -0.721, 0.7)) #1.15
        nav.locations_keys.insert(0,'rotate11')


    if str(nav.location)=='rotate2' :
        nav.locations['rotate22'] = Pose(Point(-1.327,1.008, 0.000), Quaternion(0.000, 0.000,0.9,0.45)) #0.808
        nav.locations_keys.insert(0,'rotate22')
        nav.locations['rotate222'] = Pose(Point(-2.327,1.008, 0.000), Quaternion(0.000, 0.000,0.48,0.88)) 
        nav.locations_keys.insert(0,'rotate222')

    if nav.location== f_bridge_first :
        #遍历后没找到
        # nav.locations['mf_bridge_second'] = Pose(Point(-1.9,0.03, 0.000), Quaternion(0.000, 0.000, 0,1))
        nav.locations['mf_bridge_second'] = Pose(Point(-1.9,0, 0.000), Quaternion(0.000, 0.000, 0,1))
        nav.locations['rotate1'] = Pose(Point(-1.83,-1.15, 0.000), Quaternion(0.000, 0.000, 0.346,0.94)) 
        nav.locations['rotate111'] = Pose(Point(-1.327,-1.45, 0.000), Quaternion(0.000, 0.000, 0.48,0.88)) 
        nav.locations['rotate11'] = Pose(Point(-2.55, -1.83, 0.000),Quaternion(0.000, 0.000, -0.721, 0.7)) #1.15
        nav.locations['rotate2'] = Pose(Point(-1.827,0.97, 0.000), Quaternion(0.000, 0.000, -0.95,0.315)) 
        nav.locations['rotate222'] = Pose(Point(-2.327,1.008, 0.000), Quaternion(0.000, 0.000,0.48,0.88)) 
        nav.locations['rotate22'] = Pose(Point(-1.327,1.008, 0.000), Quaternion(0.000, 0.000,0.9,0.45)) #0.808
        nav.locations_keys.insert(0,'mf_bridge_second')
        # nav.locations_keys.insert(0,'center')
        nav.locations_keys.insert(0,'rotate222')
        nav.locations_keys.insert(0,'rotate22')
        nav.locations_keys.insert(0,'rotate2')
        nav.locations_keys.insert(0,'rotate111')
        nav.locations_keys.insert(0,'rotate11')
        nav.locations_keys.insert(0,'rotate1')
        nav.move_base.cancel_goal()


def pidF2A(nav):
    # pid F->A 过桥
    rospy.set_param('nongoal_flag', 2)
    rospy.set_param("has_reached_F",2)
    nav.stop_cancel_fail = 0

def pidA2F(nav):
    """控制A到F pid 过桥的开/关"""
    if nav.location == start_t:
        rospy.set_param('start_trace', 1) 
        rospy.set_param('nongoal_flag', 2) 
    #  第一次 A->F 时开启
    if nav.location == a_bridge_first:
        # nav.rotate(1, 90)
        rospy.set_param('nongoal_flag', 3)
    if (nav.location == f_bridge_first and rospy.get_param('rescue',  0) == 4) or nav.location =='mf_bridge_second':
        rospy.set_param('nongoal_flag', 4)  #走巡线,pid过桥
        # rospy.set_param('nongoal_flag', 0)  #走巡线,导航过桥
        # if rospy.get_param('rescue',  0) != 4:
        #     rospy.set_param('take_photo', 2)   
        #     rospy.set_param('nongoal_flag', 0)  #不走巡线


        # 将导航主函数阻塞在这里，等待nongoal中pid返还控制权
    while rospy.get_param('nongoal_flag')!= 0:
        pass
    
def pose_callback(msg):
    if rospy.get_param('findboard_flag', 0) == 1:
        print('>>>pose received')
        global boardnav
        boardnav.now_position = (msg.position.x, msg.position.y)
        print((msg.position.x, msg.position.y))
        boardnav.position_initial = True


def get_pose():
    pose_sub = rospy.Subscriber('/cxy_base_link', Pose, pose_callback)

    
if __name__ == '__main__':
    
    rospy.init_node('get_boards', anonymous=True)
    get_pose()
    get_boards() 
    rospy.spin()