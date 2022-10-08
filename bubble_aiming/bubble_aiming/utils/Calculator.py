"""
:Author: HarryWen
:Date: 2022-08-18 17:55:57
FilePath: /bubble_bringup/home/nvidia/Desktop/Projects/PythonProjects/bubble/src/bubble_contrib/bubble_aiming/bubble_aiming/utils/Calculator.py
LastEditors: your name your email
LastEditTime: 2022-09-14 13:40:03
:License: GNU General Public License v3.0. See LICENSE file in root directory.
:Copyright (c) 2022 Birdiebot R&D Department
:Shanghai University Of Engineering Science. All Rights Reserved
"""


import cv2
import math
from sys import *
import numpy as np
from scipy import signal
from typing import Optional, Union


def calRectCenter(box: np.ndarray) -> np.ndarray:
    """
    Calculates the array of center points of the rectangle.

    :param box: The array of four vertices of rectangles. 
                (e.g. [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]).
    :return: Array of center points of the rectangle. 
    """

    if np.shape(box) == (4, 2):
        center = np.mean(box, axis=0)
    else:
        raise AttributeError(
            f"expected box input shape to have (4,2) but received input with shape {str(np.shape(box))}")
    return center


def calMomentCenter(contours: np.ndarray) -> np.ndarray:
    """
    Calculates the centroid of the profile.

    :param contours: Detected contours. Each contour is 
                     stored as a vector of points.
    :return: Array of centroid of the profile.
    """

    if len(contours) != 0:
        contour = max(contours, key=cv2.contourArea)
        mm = cv2.moments(contour)  # 获取轮廓的几何矩 零阶矩用于获取轮廓面积 一阶矩用于计算质心
        M00 = mm['m00']  # 零阶矩为轮廓面积
        M10, M01 = mm['m10'], mm['m01']  # 一阶矩用于计算质心
        momentCenterPoint = np.array(int(M10 / M00), int(M01 / M00))  # 几何矩中心
    else:
        raise AttributeError("expected contours input length > 0")
    return momentCenterPoint


def calLengthWidthRatio(box: np.ndarray) -> float:
    """
    Calculates the length-width ratio of the rectangle. 
    By default, the longer side is the length of the rectangle.  

    :param box: The array of four vertices of rectangles. 
                (e.g. [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]).
    :return: Length-width ratio of the rectangle.
    """

    if np.shape(box) == (4, 2):
        len1 = calPointDistance(box[0], box[2])
        len2 = calPointDistance(box[0], box[1])
        height, weight = min([len1, len2]), max([len1, len2])
        try:
            ratio = weight / height  # 长宽比
        except Exception:
            raise ZeroDivisionError
    else:
        raise AttributeError(
            f"expected box input shape to have (4,2) but received input with shape {str(np.shape(box))}")
    return ratio


def calPointDistance(start_point: np.ndarray, end_point: np.ndarray) -> float:
    """
    Calculate the Euclidean distance between two points in
    n-dimensional space, i.e. the absolute distance.

    :param start_point: The array of line start points, 
                        (e.g. np.array((10, 10), dtype=int)).
    :param end_point: The array of line end points (e.g. np.array((10, 10), dtype=int)).
    :return: Euclidean distance between two points.
    """

    if np.shape(start_point) == np.shape(end_point):
        distance = np.linalg.norm(start_point - end_point)
    else:
        raise AttributeError(
            f"""It is expected that the start point and the end point have the same input shape,but the shape of 
                start_point is {str(np.shape(start_point))}, and the shape of end_point is {str(np.shape(end_point))}.""")

    return distance


def calLineRadians(start_point: np.ndarray, end_point: np.ndarray) -> float:
    """
    A rectangular coordinate system is established with 
    the end point of the straight line as the origin. 
    The angle (radian) between the straight line and the 
    X axis is calculated on the basis of this coordinate system.

    :param start_point: The array of line start points (e.g. np.array((10, 10), dtype=int)).
    :param end_point: The array of line end points (e.g. np.array((10, 10), dtype=int)).
    :return: Angle between line and coordinate axis (radians).
    """

    h, w = start_point - end_point
    rad = math.atan2(h, w)
    return rad


def calAngularVelocity(last_rad: float, cur_rad: float, time_differ: float) -> float:
    """
    Angular velocity is calculated by angle(radians) 
    difference and time interval.

    :param last_rad: Float number of last angle(radians).
    :param cur_rad: Float number of the current angle(radians).
    :param time_differ: Float number of the time difference(seconds) 
                        between last_rad and cur_rad.
    :return: Angular(radians) velocity.

    """

    radian_differ = calRadianGap(cur_rad, last_rad)
    AngleVelo = radian_differ / time_differ
    return AngleVelo


def findPeak(data_array: np.ndarray, thres: float) -> Optional[int]:
    """
    Find peaks inside a signal based on peak properties.Returns 
    the peak with the largest value among the qualified peaks

    :param data_array: Array_like or scalar. A signal with peaks.
    :param thres: Float number of peak threshold.
    :return: Peak index in data_array. If there is no qualified 
             peak in the data, return none.
    """

    peak_index = None
    peaks_index_list, peaks_dict = signal.find_peaks(
        data_array, height=thres, distance=20)  # 可以研究一下他的可选参数
    if len(peaks_index_list) != 0:
        peak_index = np.argmax(peaks_dict["peak_heights"])
        if 20 >= peak_index or peak_index >= len(data_array) - 20:
            peak_index = None
    return peak_index


def findTrough(data_array: np.ndarray, thres: float) -> Optional[int]:
    """
    Find troughs inside a signal based on trough properties.
    Returns the trough with the smallest value among the qualified troughs

    :param data_array: Array_like or scalar. A signal with troughs.
    :param thres: Float number of trough threshold.
    :return: Trough index in data_array. If there is no qualified trough 
             in the data, return none.
    """

    trough_index = None
    negative_data_array = np.negative(data_array)  # 数组取相反数
    troughs_index_list, troughs_dict = signal.find_peaks(
        negative_data_array, height=-thres, distance=20)
    if len(troughs_index_list) != 0:
        trough_index = np.argmin(troughs_dict["peak_heights"])
        if 20 >= trough_index or trough_index >= len(data_array) - 20:
            trough_index = None
    return trough_index


def getRectMaxLenPoint(point_list: list) -> list:
    """
    用于获取矩形长边的点坐标
    :param point_list:
    :return: 矩形长边的两点坐标
    """
    dis_list = []
    for i in range(1, 4):
        dis = calPointDistance(point_list[0], point_list[i])
        dis_list.append(dis)
    sort_dis_list = sorted(dis_list)
    max_len_point = [point_list[0],
                     point_list[dis_list.index(sort_dis_list[1]) + 1]]
    return max_len_point


def rigidTransform(center: np.ndarray, theta: float, point_list: Union[np.ndarray, list]) -> np.ndarray:
    """
    Rigid body transformation of polygon in 2D plane 

    :param center: The array of polygon rotation center.
    :param theta: Polygon rotation angle (angles) around the center. 
                  Positive values mean counter-clockwise rotation 
                  (the coordinate origin is assumed to be the top-left corner).
    :param point_list: The array or list of polygon vertex.
    :return: Polygon vertices after rigid body transformation.
    """

    new_point_list = []
    center = list(map(int, center))
    # 获取仿射变换矩阵，由于没有进行放缩，故可以认为是刚体变换矩阵
    rotation_matrix = cv2.getRotationMatrix2D(center, theta, 1)
    for point in point_list:
        point = np.append(point, [1])
        new_point = np.rint(np.dot(rotation_matrix, point))  # 四舍五入
        new_point_list.append(new_point.tolist())
    new_point_list = np.array(new_point_list)
    return new_point_list


def sortCoordinate(point_list: np.ndarray) -> np.ndarray:
    """
    对矩形框四点坐标进行排序，顺序为 左上，右上，右下，左下
    :param point_list: 矩形四点坐标
    :return: 顺时针排序的矩形四点坐标
    """
    point_list = np.array(point_list)
    center = calRectCenter(point_list)
    max_len_point = getRectMaxLenPoint(point_list)
    degree = math.degrees(calLineRadians(max_len_point[0], max_len_point[1]))
    rigid_point = rigidTransform(center, degree, point_list)
    temp = rigid_point[:, 0] + rigid_point[:, 1]
    rigid_point_list = []
    for i in range(4):
        if i != np.argmin(temp) and i != np.argmax(temp):
            rigid_point_list.append(rigid_point[i])
    rigid_point = np.array(rigid_point_list)

    up_left = rigid_point[np.argmin(temp)]
    down_right = rigid_point[np.argmax(temp)]
    down_left = rigid_point[np.argmin(rigid_point[:, 0])]
    up_right = rigid_point[np.argmax(rigid_point[:, 0])]

    new_point_list = [up_left, up_right, down_right, down_left]
    new_rigid_point = rigidTransform(center, -degree, new_point_list)
    return new_rigid_point


def calIOU(box_rotation_info1, box_rotation_info2):
    """比较图像框交并补"""
    iou = 0
    area_ratio = 0
    int_pts = cv2.rotatedRectangleIntersection(
        tuple(box_rotation_info1), tuple(box_rotation_info2))[1]
    if int_pts is not None:
        area1, area2 = box_rotation_info1[1][0] * \
            box_rotation_info1[1][1], box_rotation_info2[1][0] * \
            box_rotation_info2[1][1]
        order_pts = cv2.convexHull(int_pts, returnPoints=True)
        int_area = cv2.contourArea(order_pts)
        iou = int_area / (area1 + area2 - int_area)
        area_ratio = max(area1, area2)/min(area1, area2)
    if area_ratio > 2:
        iou = 0
    return iou


def calRadianGap(radian1, radian2):
    if np.sign(radian1) != np.sign(radian2):
        abs_radian1 = abs(radian1)
        abs_radian2 = abs(radian2)
        if abs(radian1) > 1.5708:
            gap = 6.28319 - (abs_radian1 + abs_radian2)
        else:
            gap = abs_radian1 + abs_radian2
    else:
        gap = abs(radian1 - radian2)
    return gap


def calOrthogonalVelocity(point1: tuple, point2: tuple, time_gap: float):
    """计算直线正交分解后的线速度

    :param point1: 点 1
    :param point2: 点 2
    :param time_gap: 时间差
    :return: 延x轴和y轴方向的线速度
    """
    x_vel = (point1[0]-point2[0]) / time_gap
    y_vel = (point1[1]-point2[1]) / time_gap
    return x_vel, y_vel
