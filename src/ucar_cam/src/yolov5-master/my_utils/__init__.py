import os
import cv2
import numpy as np

"""
罗鑫自建工具包
"""


def free_dir(path):
    """
    清理目录
    """
    if not os.path.exists(path):  # 如果目录不存在就创建目录
        os.mkdir(path)
    else:  # 如果目录存在就清除目录
        del_files(path)


def del_files(path_file):
    """
    删除目录下所有文件
    """
    ls = os.listdir(path_file)
    for i in ls:
        f_path = os.path.join(path_file, i)
        # 判断是否是一个目录,若是,则递归删除
        if os.path.isdir(f_path):
            del_files(f_path)
        else:
            os.remove(f_path)


def drawtext(image, pt1, pt2, text) -> None:
    """
    在图片上绘制预测框
    Args:
        image: 图片
        pt1: 预测框左上角坐标
        pt2: 预测框右下角坐标
        text: 文本
    """
    fontFace = cv2.FONT_HERSHEY_COMPLEX_SMALL  # 这个字体便于绘制小尺寸的字
    fontScale = 0.8  # 字体大小
    line_thickness = 3  # 线条粗细
    font_thickness = 1  # 文字笔画粗细
    line_back_color = (0, 0, 255)  # 线条和文字背景框颜色：红色
    font_color = (255, 255, 255)  # 文字颜色：白色

    # 绘制矩形框
    cv2.rectangle(image, pt1, pt2, color=line_back_color, thickness=line_thickness)
    # 计算文本宽高：retval:文本宽高；_处是baseLine:基线与最低点之间的距离
    retval, _ = cv2.getTextSize(text=text, fontFace=fontFace, fontScale=fontScale,
                                thickness=font_thickness)  # 输入文本的属性信息，输出文本形状高宽信息
    # 计算覆盖文本的矩形坐标(会有一个矩形把文本框起来)
    top_left = (pt1[0], pt1[1] - retval[1])  # 基线与目标框的上边缘重合
    bottom_right = (top_left[0] + retval[0], top_left[1] + retval[1])
    # 绘制文字的背景框
    cv2.rectangle(image, top_left, bottom_right, thickness=-1, color=line_back_color)

    # 绘制纯文本
    cv2.putText(image, text, pt1, fontScale=fontScale,  # pt1表示文字左上角坐标
                fontFace=fontFace, color=font_color, thickness=font_thickness)


def point_in_convex_polygon(point, polygon):
    """
    判断点是否在四边形中
    """
    # 将四边形的四个顶点坐标分别存储在四个变量中
    x1, y1, x2, y2, x3, y3, x4, y4 = polygon
    # 计算点与矩形四个顶点所形成的四个三角形的叉积
    cp1 = np.cross([x2 - x1, y2 - y1], [point[0] - x1, point[1] - y1])
    cp2 = np.cross([x3 - x2, y3 - y2], [point[0] - x2, point[1] - y2])
    cp3 = np.cross([x4 - x3, y4 - y3], [point[0] - x3, point[1] - y3])
    cp4 = np.cross([x1 - x4, y1 - y4], [point[0] - x4, point[1] - y4])
    # 如果四个叉积的符号都相同，则点在四边形内部
    if cp1 * cp2 >= 0 and cp2 * cp3 >= 0 and cp3 * cp4 >= 0:
        return True
    else:
        return False


def calculate_iou(rect1, rect2):
    """
    输入两个矩形的左上角和右下角坐标
    计算交并比
    """
    # 计算交集区域的左上角和右下角坐标
    intersection_left = max(rect1[0], rect2[0])
    intersection_top = max(rect1[1], rect2[1])
    intersection_right = min(rect1[2], rect2[2])
    intersection_bottom = min(rect1[3], rect2[3])

    # 计算交集区域的宽度和高度
    intersection_width = max(0, intersection_right - intersection_left)
    intersection_height = max(0, intersection_bottom - intersection_top)

    # 计算交集区域的面积
    intersection_area = intersection_width * intersection_height

    # 计算两个矩形的面积
    rect1_area = (rect1[2] - rect1[0]) * (rect1[3] - rect1[1])
    rect2_area = (rect2[2] - rect2[0]) * (rect2[3] - rect2[1])

    # 计算并集区域的面积
    union_area = rect1_area + rect2_area - intersection_area

    # 计算 IoU
    iou = intersection_area / union_area

    return iou

