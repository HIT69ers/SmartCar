#!/usr/bin/env python 
# -*- coding: utf-8 -*-
"""
v 1.0
@date: 2023.7.3
@auther: 陈笑阳
v 2.0
@date: 2024.5.1
@auther: 吴相军
"""

import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from Utils import Board, nearby_2d, get_point
import math
from sensor_msgs.msg import LaserScan
import geometry_msgs
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
MARGIN = 0.27 # 板子周围的余量
RANGE  = 0.4 # 拍照距离板子的距离 0.5
LENGTH = 0.75 # 中垂线拍照距离



'''
    坐标变换找板子
'''
camera_matrix = np.array([[219.9364,0,169.1259], [0, 220.3548, 119.1613], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([-0.3845,0.1324,0.0,0.0,0.0], dtype=np.float64)

R_radar_to_cam = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0]
    ])

t_radar_to_cam = np.array([0.0, -0.04, -0.25]).reshape(3, 1)
def rpy2quaternion(roll, pitch, yaw):
    x=math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2)-math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    y=math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)+math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
    z=math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)-math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
    w=math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2)+math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    return np.array([x, y, z, w ])
class lidar2cam:
    def __init__(self, camera_matrix, dist_coeffs):
        camera_matrix = np.array([[219.9364,0,169.1259], [0, 220.3548, 119.1613], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.array([-0.3845,0.1324,0.0,0.0,0.0], dtype=np.float64)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.cloud_points = []
        self.kb = []
        self.goal = []
        self.centor= []
        self.position = np.array([0, 0])
        self.car_position = np.array([0, 0, 0])
        self.dis = 0.8 # m


    def transform(self, camera_points):
        """
        相机坐标系转像素坐标系
        """
        self.camera_points = camera_points
        R = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]], dtype=np.float32)
        tvecs = np.array([0, 0, 0], dtype=np.float32)
        rvecs, _ = cv2.Rodrigues(R)
        image_points, _ = cv2.projectPoints(camera_points, rvecs, tvecs, self.camera_matrix, self.dist_coeffs)
        # print("image_points are ", image_points)
        self.image_points = image_points


        return image_points
    def filter(self, bounding_boxs):
        """
        经bounding_box后筛选的点云数据
        """
        x_min = bounding_boxs.x - bounding_boxs.w / 2
        x_max = bounding_boxs.x + bounding_boxs.w / 2
        np.array(x_min)
        np.array(x_max)
        
        image_points_x =  self.image_points[:, :, 0].reshape(-1)
        # print("image_points shape is", self.image_points.shape)
        self.filter_cloud_points = self.cloud_points[(x_min < image_points_x) & (image_points_x < x_max)]
        print("cloud size", self.cloud_points.shape[0], "filter_size", self.filter_cloud_points.shape[0])
        # print("filter_points is ", self.filter_cloud_points)
        # self.filter_cloud_points = [[x / 2 for x in sublist] for sublist in self.filter_cloud_points]
        # self.filter_cloud_points = self.filter_cloud_points / 2
        print("筛选的点云数据cloud is", self.filter_cloud_points)  # check clouds

    def compute_kb(self):
        """
        计算识别板的中心和法线
        """
        n = self.filter_cloud_points.shape[0]
        sum_xy = np.sum(self.filter_cloud_points[:, 0] * self.filter_cloud_points[:, 1])
        sum_x2 = np.sum(self.filter_cloud_points[:, 0] * self.filter_cloud_points[:, 0])
        x_mean = np.mean(self.filter_cloud_points[:, 0])
        y_mean = np.mean(self.filter_cloud_points[:, 1])
        k_ = - 1 / ((sum_xy - n * x_mean * y_mean) / (sum_x2 - n * x_mean * x_mean))
        b_ = y_mean - k_ * x_mean
        self.kb = [k_, b_]
        self.centor = [x_mean, y_mean]
        print("k,b is", self.kb, self.centor)

    def compute_goal(self):
        """
        判断具体应该走哪个点，目前逻辑是走两个中最近的
        """
        dx = math.pow(self.dis * self.dis / ((self.kb[0] * self.kb[0]) + 1), 1/2)
        dy = self.kb[0] * dx
        point1 = np.array([self.centor[0] + dx, self.centor[1] + dy])
        point2 = np.array([self.centor[0] - dx, self.centor[1] - dy])
        diff1 = self.position - point1
        diff2 = self.position - point2
        distances1 = np.linalg.norm(diff1)
        distances2 = np.linalg.norm(diff2)
        if distances1 > distances2:
            self.goal = point2
        else:
            self.goal = point1
            
        x = self.goal[0]
        y = self.goal[1]
        self.goal = [ 0.707*x + 0.707*y,-0.707*x + 0.707*y]
        return self.goal

    def solve(self, cloud_points, bounding_boxs, camera_points):
        self.cloud_points = cloud_points
        self.transform(camera_points)
        self.filter(bounding_boxs)
        self.compute_kb()
        self.compute_goal()
        print("goal is", self.goal)


model = lidar2cam(camera_matrix, dist_coeffs)

class bounding_box:
    def __init__(self,cls, x, y, w, h):
        self.cls = cls
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        
# def laser_callback(msg):


"""
calculate_rectangle_properties(rectangle)
----------------------------
参数：rectangle
@rectangle:所需框的所有信息 1*6
----------------------------
返回值 x, y, w, h
@x, y, w, h：为传入框的中心点，宽高

"""
def calculate_rectangle_properties(rectangle):
    x1, y1, x2, y2,_,__ = rectangle
    x = (x1 + x2) / 2
    y = (y1 + y2) / 2
    w = x2 - x1
    h = y2 - y1

    
    return x, y, w, h
def pose_callback(msg):
    global box_flag, pose_flag

    print(model.car_position)

def FindBoard_M2():


    R_radar_to_cam = np.array([
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0]
        ])

    t_radar_to_cam = np.array([0.0, -0.04, -0.25]).reshape(3, 1)
    new_position = Board()
    print("反复运行？？？")
    # msg = rospy.Subscriber("/scan", LaserScan,queue_size=1)
    msg = rospy.wait_for_message('/scan', LaserScan,  timeout=None)
    car_msg = rospy.wait_for_message("/cxy_base_link", Pose, timeout = 5)
    theta = euler_from_quaternion([car_msg.orientation.x, car_msg.orientation.y, car_msg.orientation.z, car_msg.orientation.w])[2]
    model.car_position = np.array([car_msg.position.x, car_msg.position.y, theta])
    # 首先进行视觉的处理
    bound = bounding_box(0,0,0,0,0)
    x1 = rospy.get_param('result_cam_0', 0)
    y1 = rospy.get_param('result_cam_1', 0)
    x2 = rospy.get_param('result_cam_2', 0)
    y2 = rospy.get_param('result_cam_3', 0)
    bound.x = (x1 + x2) / 2
    bound.y = (y1 + y2) / 2
    bound.w = x2 - x1
    bound.h = y2 - y1
    print(f"x:{bound.x}")
    print(f"y:{bound.y}")
    print(f"w:{bound.w}")
    print(f"h:{bound.h}")    
    # print("msg is ", msg)
    # bound.x = 146.5
    # bound.y = 96.0
    # bound.w = 37.0
    # bound.h = 40.0
    # print()
    # print(f"x:{bound.x}")
    # print(f"y:{bound.y}")
    # print(f"w:{bound.w}")
    # print(f"h:{bound.h}")    
    print("已进入紧急找板程序！！！")
    points_new = np.array([]).reshape(0, 3)
    point_plot_new = np.array([]).reshape(0, 2)
    camera_points = np.array([]).reshape(0, 3)
    # with open('/home/ucar/ucar_ws/src/ucar_nav/scripts/msg_ranges.txt','w') as f:
    #     f.write(str(msg.ranges.tolist()))
    print(f"msg.range_min:{msg.range_min}")
    print(f"msg.angle_increment:{msg.angle_increment}")
    for i in range(len(msg.ranges)):
    
        if msg.ranges[i] != msg.range_min:
        
            angle = msg.angle_min + i * msg.angle_increment
            x1 = msg.ranges[i] * np.cos(angle)
            y1 = msg.ranges[i] * np.sin(angle) 
            x = 0.707*x1 - 0.707*y1
            y = 0.707*x1 + 0.707*y1
            # print("angle is", angle)
            r = pow(x**2 + y**2, 1/2)
            # print(r)
            if x == float('nan') or y == float('nan') or r < 0.1 or abs(angle) < 1.17 or y>0:
                continue
            point = np.array([[-1*y, -1*x, 0]])
            point_plot = np.array([[-1*y, -1*x]])
            points_new = np.append(points_new, point, axis=0)
            point_plot_new = np.append(point_plot_new , point_plot, axis=0)
            tmp = point.reshape(3,1)
            tmp_cam = np.dot(R_radar_to_cam, tmp) + t_radar_to_cam
            camera_points = np.append(camera_points, tmp_cam.reshape(1, 3),axis=0)
    # print("cloud_points is ", points_new)
    # print("camera_points is ", camera_points)
    # print("bound is ", bound)
    with open('/home/ucar/ucar_ws/src/ucar_nav/scripts/new_points.txt','w') as f:
        f.write(str(point_plot_new.tolist()))
    model.solve(points_new, bound, camera_points)

    R = np.array([
        [np.cos(model.car_position[2]), -np.sin(model.car_position[2]), 0],
        [np.sin(model.car_position[2]),  np.cos(model.car_position[2]), 0],
        [0,             0,             1]
    ])
    goal_position = np.array([model.goal[0], model.goal[1], 0])
    centor_position = np.array([model.centor[0], model.centor[1], 0])
    
    
    car_position = np.array([model.car_position[0], model.car_position[1], 0])
    print(f"car_position:{car_position}")
    goal_map_position = np.dot(R, goal_position.reshape(3,1)) + car_position.reshape(3,1)
    print(f"goal_map_position:{goal_map_position}")
    centor_map_position = np.dot(R, centor_position.reshape(3,1)) + car_position.reshape(3,1)
    print(f"centor_map_position:{centor_map_position}")
    theta1 = 0
    dis  = centor_map_position - goal_map_position
    print(f"dis:{dis}")
    if dis[0] > 0 and dis[1] > 0:
        theta1 = np.arctan(dis[1] / dis[0])
    elif dis[0] < 0 and dis[1] > 0:
        theta1 = np.arctan(dis[1] / dis[0]) + np.pi
    elif dis[0] < 0 and dis[1] < 0:
        theta1 = np.arctan(dis[1] / dis[0]) - np.pi
    elif dis[0] > 0 and dis[1] < 0:
        theta1 = np.arctan(dis[1] / dis[0])
    quaternion = rpy2quaternion(0, 0, theta1)
    print(f"quaternion:{quaternion}")
    print(f"Pose:{Pose(Point(goal_map_position[0], goal_map_position[1], 0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))}")
    # rospy.Subscriber("/scan", LaserScan, laser_callback).unregister()
    return Pose(Point(goal_map_position[0], goal_map_position[1], 0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

    # return new_position.aim_pose
        
'''
    传统方法找板子
'''
def euclidean_distance(vector1, vector2):
    return np.linalg.norm(np.array(vector1) - np.array(vector2))
class BoardNav():
    def __init__(self) -> None:
        self.resolution = 0.05  # 地图分辨率
        self.length1 = 80 #100
        self.length2 = 40
        self.img = []
        self.now_position = ()
        self.position_initial = False
        self.map_initial = False
        self.goals = []
        self.F_center = np.array([-1.85, -0.42])
        self.Pic_center = np.array([-1.85, -0.42])
        # self.client_lci = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer")
        # 仅保留 filter[0]*filter[0]大小的框不能包含 且 filter[1]*filter[1]大小的框能包含 的框
        self.area = (4, 16) 

    def extractPointsFromPic(self):
        img = self.img
        img = np.where(img >99, 255, img) 
        img = np.where(img <100, 0, img) 
        # 将图像最外一圈边框设为黑色
        img[0:2, : ] = 0
        img[78:81, :] = 0
        img[:, 0:2] = 0
        img[:, 37:41] = 0
        img[40:41:, 24:40] = 0
        img[50:51:, 24:40] = 0
        num_row=80
        num_col=40
        num_thresh1 = 21
        num_thresh2=21


        # img = cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        ret, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        for i in range(num_row//2, 0, -1):
            if np.count_nonzero(img[i, :] == 255) > num_thresh2:
                thresh[:i, :] = 255
                break
            
        for i in range(num_row//2, num_row):
            if np.count_nonzero(img[i, :] == 255) > num_thresh2:
                thresh[i:num_row, :] = 255
                break
            
        for i in range(num_col//2, 0, -1):
            if np.count_nonzero(img[:, i] == 255) > num_thresh1:
                thresh[:,0:i] = 255
                break
            
        for i in range(num_col//2, num_col):
            if np.count_nonzero(img[:, i] == 255) > num_thresh1:
                thresh[:, i:num_col] = 255
                break

        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        points = []
        new_contours = []
        # print(thresh)
        with open('/home/ucar/ucar_ws/src/ucar_nav/scripts/thresh.txt','w') as f:
            f.write(str(thresh.tolist()))
        for i, contour in enumerate(contours):
            if hierarchy[0][i][3] == -1:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            central_point = (x+w/2, y+h/2)
            if (w < self.area[1] and h < self.area[1]) and (w > self.area[0] or h > self.area[0]):
                points.append(central_point)
                new_contours.append(contour)
        print(f"障碍物的中心点为{points}\n")
        return points, new_contours


    def findboard(self,msg):
        points, new_contours = self.extractPointsFromPic()
        points_num = len(points)
        
        try:
            board_positions = self.fake2real(points)
            print(f'障碍板中心在地图中位置为:\n{board_positions}\n')
            dirs = []
            print(len(new_contours))
            for contour in new_contours:
                points = self.gen_points(contour)
                dir = self.get_dir(points)
                dirs.append(dir)
            # 形成障碍板列表
            boards = []
            angles = np.arctan2(-board_positions[:, 1] + self.F_center[1], -board_positions[:, 0] + self.F_center[0])      
            for  board_position, angle, dir in zip(board_positions, angles, dirs):
                
                board = Board()
                board.position = board_position
                # print(f'板子朝向的角度为{np.rad2deg(angle)}')
                board.dir = np.array([np.cos(angle), np.sin(angle)])
                tmp_angle = np.arctan2(dir[1], dir[0])
                print(np.rad2deg(tmp_angle))
                tmp_angle1 = tmp_angle + np.pi / 2
                tmp_angle2 = tmp_angle - np.pi / 2
                # print(np.rad2deg(tmp_angle1),np.rad2deg(tmp_angle1),)
                # 选取与板子与中心连线夹角在-90～90度的角
                # print( np.dot(board.dir , np.array([np.cos(tmp_angle1),np.sin(tmp_angle1)])))
                if np.dot(board.dir , np.array([np.cos(tmp_angle1),np.sin(tmp_angle1)])) > 0:  
                    tmp_angle = tmp_angle1
                    # print('a')
                else:
                    tmp_angle = tmp_angle2
                    # print('b')
                board.dir = np.array([np.cos(tmp_angle), np.sin(tmp_angle)])
                print(f'板子{board.position}朝向的角度为{np.rad2deg(tmp_angle)}')
                # 若找到两个板子则使用板子朝向设置小车方向，否则使用房间中心与板子的连线方向设置小车朝向
                # if points_num == 2: 
                #     board.dir = np.array([np.cos(tmp_angle), np.sin(tmp_angle)])
                #     print(f'板子朝向的角度为{np.rad2deg(np.arctan2(np.sin(tmp_angle),np.cos(tmp_angle)))}')
                boards.append(board)
            
            for board in boards:
                
                
                tmp_point = get_point(board.position, board.dir, RANGE)
                tmp_point_1 = get_point(board.position, np.array([-board.dir[0],-board.dir[1]]), RANGE-0.15)
                # 判断目标点分别与车点的距离
                # 选取与车欧式距离比较近的点
                print(f"车的位置是{[msg.position.x,msg.position.y]}")
                print(f"tmp_point是{tmp_point}")
                print(f"tmp_point_1是{tmp_point_1}")
                if euclidean_distance(tmp_point_1,[msg.position.x,msg.position.y]) < euclidean_distance(tmp_point,[msg.position.x,msg.position.y]) and (tmp_point_1[0]<-0.75 and tmp_point_1[0]>-2.75 and tmp_point_1[1]<1.75 and tmp_point_1[1]>-2.25):
                    tmp_point = tmp_point_1
                    board.dir = - board.dir
                    print(f"tmp_point已替换!!!")
                exist_flag = 0
                for board2 in boards:
                    if board == board2:
                        continue
                    if nearby_2d(tmp_point, board2.position, MARGIN):
                        exist_flag = 1
                        print('从垂直平分线上找点')
                        center = (board.position + board2.position) / 2
                        # 垂直平分线的方向向量
                        angle = np.arctan2((board2.position - board.position)[1], (board2.position - board.position)[0])
                        angle1 = angle + np.pi / 2
                        angle2 = angle - np.pi / 2
                        # 选取与板子法向量夹角在-90～90度的角
                        if np.dot(board.dir , np.array([np.cos(angle1),np.sin(angle1)])) > 0:  
                            angle = angle1
                        else:
                            angle = angle2
                        dir = np.array([np.cos(angle), np.sin(angle)])
                        # 垂直平分线上的点
                        tmp_len = np.sqrt(LENGTH ** 2 - (board.position[0] - center[0]) ** 2 - (board.position[1] - center[1]) ** 2)
                        print(f'tem_len is {tmp_len}')
                        aim_point = dir * tmp_len + center
                        board.gen_pose(aim_point, (- np.cos(angle), - np.sin(angle)))
                        board.uni_shot = True
                        break
                if exist_flag == 0:
                    print('余量点不在任意其他板子的范围内')
                    board.gen_pose(tmp_point, - board.dir)
            return boards
        except:
            print('未探测到障碍物')
            return []
    
    def fake2real(self, points):
        return self.resolution * np.array(points) + self.Pic_center - self.resolution * np.array([self.length2/2, self.length1/2])
    
    def rotate_strategy(self):
        points, new_contours = self.extractPointsFromPic()
        try:
            board_positions = self.fake2real(points)
            print(f'障碍板中心在地图中位置为:\n{board_positions}\n')
            # 形成障碍板列表
            boards = []
            # F房间中心与板子中心的夹角
            angles = np.arctan2(-board_positions[:, 1] + self.F_center[1], -board_positions[:, 0] + self.F_center[0])
            for  board_position, angle in zip(board_positions, angles):
                board = Board()
                board.position = board_position
                print(f'板子朝向的角度为{np.rad2deg(angle)}')
                board.dir = np.array([np.cos(angle), np.sin(angle)])
                boards.append(board)
            print()
            for board in boards:
                board.gen_pose(self.F_center, -board.dir)
            return boards
        except:
            print('未探测到障碍物')
            return []

    def gen_points(self, contour):
        # 取出轮廓线上的点集合
        ps = []
        for item in contour:
            ps.append(item[0].tolist())
        ps = self.fake2real(np.array(ps))
        return ps
    
    def get_dir(self, points):
        points = np.array(points)
        # print(len(points))
        distances = np.linalg.norm(points[:, np.newaxis, :] - points, axis=2)
        max_distance_index = np.unravel_index(np.argmax(distances), distances.shape)
        # print(max_distance_index[0],max_distance_index[1])
        # print(points[max_distance_index[0]])
        # print(points[max_distance_index[1]])
        # 画出最远的两个点
        # cv2.circle(img, tuple(points[max_distance_index[0]]), 1, (0, 0, 255), -1)
        # cv2.circle(img, tuple(points[max_distance_index[1]]), 1, (0, 0, 255), -1)
        # 画出最远两点的连线
        # cv2.line(img, tuple(points[max_distance_index[0]]), tuple(points[max_distance_index[1]]), (0, 0, 255), 1)
        # 返回两点形成的向量
        return np.array(points[max_distance_index[0]]) - np.array(points[max_distance_index[1]])
    
    def get_central_square(self):
        square = np.array([(26,24),(14,24),(14,36),(26,36)]) # 右上角为原点
        return self.fake2real(square)


# def pose_callback(msg):
#     if rospy.get_param('findboard_flag', 0) == 1:
#         print('>>>pose received')
#         global boardnav
#         boardnav.now_position = (msg.position.x, msg.position.y)
#         print((msg.position.x, msg.position.y))
#         boardnav.position_initial = True


# def get_pose():
#     pose_sub = rospy.Subscriber('/cxy_base_link', Pose, pose_callback)
#     rospy.spin()

if __name__ == '__main__':
    boardnav = BoardNav()
    # inflation_radius = boardnav.client_lci.get_configuration()['inflation_radius']
    # thread2 = threading.Thread(target=get_pose)
    # thread2.start()
    while not rospy.is_shutdown():
        # boardnav.client_lci.update_configuration({'inflation_radius' : 0.05})
        rospy.loginfo('enter!!!!')
        position = rospy.wait_for_message('/cxy_base_link', Pose, timeout = 5)
        boardnav.now_position = (position.position.x, position.position.y)
        data = rospy.wait_for_message('/move_base/local_costmap/costmap', OccupancyGrid, timeout = 5)
        rospy.loginfo((position.position.x, position.position.y))
        boardnav.width = data.info.width
        boardnav.height = data.info.height
        boardnav.resolution = data.info.resolution
        boardnav.img = data.data
        # rospy.set_param('findboard_flag', 0)
        boardnav.nav()
        # 回复修改的局部膨胀层半径
        # boardnav.client_lci.update_configuration({'inflation_radius' : inflation_radius})
        # rospy.set_param('findboard_flag', 2)
        rospy.signal_shutdown('task finished')