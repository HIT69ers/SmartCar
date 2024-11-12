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


MARGIN = 0.27 # 板子周围的余量
RANGE  = 0.5 # 拍照距离板子的距离
LENGTH = 0.75 # 中垂线拍照距离


class BoardNav():
    def __init__(self) -> None:
        self.resolution = 0.01  # 地图分辨率
        self.length1 = 400 #100
        self.length2 = 200
        self.img = []
        self.now_position = ()
        self.position_initial = False
        self.map_initial = False
        self.goals = []
        self.F_center = np.array([-1.85, -0.42])
        self.Pic_center = np.array([-1.85, -0.42])
        # self.client_lci = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer")
        # 仅保留 filter[0]*filter[0]大小的框不能包含 且 filter[1]*filter[1]大小的框能包含 的框
        self.area = (150, 750)#6，30 

    def extractPointsFromPic(self):
        img = self.img
        img = np.where(img >99, 255, img) 
        img = np.where(img <100, 0, img) 
        # 将图像最外一圈边框设为黑色
        # img[0:1, : ] = 0
        # img[79:80, :] = 0
        # img[:, 0:1] = 0
        # img[:, 39:40] = 0
        # img[37:38:, 25:40] = 0
        # img[47:49:, 25:40] = 0
        num_row=400
        num_col=200
        num_thresh1 = 105
        num_thresh2=105


        # img = cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        ret, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        for i in range(num_row//2, 1, -1):
            if np.count_nonzero(img[i, :] == 255) > num_thresh2:
                thresh[0:i, :] = 255
                break
            
        for i in range(num_row//2, num_row):
            if np.count_nonzero(img[i, :] == 255) > num_thresh2:
                thresh[i:num_row, :] = 255
                break
            
        for i in range(num_col//2, 1, -1):
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


    def findboard(self):
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
                # print(np.rad2deg(tmp_angle))
                tmp_angle1 = tmp_angle + np.pi / 2
                tmp_angle2 = tmp_angle - np.pi / 2
                # print(np.rad2deg(tmp_angle1),np.rad2deg(tmp_angle1),)
                # 选取与板子与中心连线夹角在-90～90度的角
                # print( np.dot(board.dir , np.array([np.cos(tmp_angle1),np.sin(tmp_angle1)])))
                if np.dot(board.dir , np.array([np.cos(tmp_angle1),np.sin(tmp_angle1)])) > 0:  
                    tmp_angle = tmp_angle1
                else:
                    tmp_angle = tmp_angle2
                board.dir = np.array([np.cos(tmp_angle), np.sin(tmp_angle)])
                # print(f'板子朝向的角度2为{np.rad2deg(tmp_angle)}')
                # 若找到两个板子则使用板子朝向设置小车方向，否则使用房间中心与板子的连线方向设置小车朝向
                # if points_num == 2: 
                #     board.dir = np.array([np.cos(tmp_angle), np.sin(tmp_angle)])
                #     print(f'板子朝向的角度为{np.rad2deg(np.arctan2(np.sin(tmp_angle),np.cos(tmp_angle)))}')
                boards.append(board)
            print()
            for board in boards:
                tmp_point = get_point(board.position, board.dir, RANGE)
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