'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-17 02:23:17
FilePath: /bubble/src/bubble_contrib/bubble_debuger/bubble_debuger/point_debugger.py
LastEditors: Ligcox
LastEditTime: 2022-07-06 20:10:15
E-mail: robomaster@birdiebot.top
'''
from bboxes_ex_msgs import msg
import rclpy
from rclpy.node import Node

import message_filters

import bboxes_ex_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from bboxes_ex_msgs.msg import BoundingPolygonBoxes2D, BoundingBoxes2D
from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped

import cv2


# 海康相机
Camera_intrinsic = {
    "mtx": np.array([[1315.218838,0.000000,653.391116],
                     [0.000000,1314.338369,469.761802],
                     [0.000000,0.000000,1.000000]],
                    dtype=np.double),
    "dist": np.array([-0.086407,0.142111,-0.001130,-0.001981,0.000000], dtype=np.double),
}
# Camera_intrinsic = {
#     "mtx": np.array([[1.32280384e+03, 0.00000000e+00, 6.00000000e+02],
#                      [0.00000000e+00, 1.32691122e+03, 5.0000000e+02],
#                      [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]],
#                     dtype=np.double),
#     "dist": np.array([2.50140551e-03, -3.05940539e+00, -9.93333169e-03,
#                       -3.64458084e-04, 2.33302573e+01], dtype=np.double),
# }




armourSize = {
    "bigArmour": [0.235, 0.058],
    "normalArmour": [0.140, 0.058],
    "energyArmour": [0.230, 0.127]
}

from sys import *
import cv2
import time
import math
import numpy as np
from itertools import groupby
from scipy import signal
from copy import deepcopy


# TODO 后面重写
def calGyroBoxPoint(average_mid_point, box_width, box_height):
    '''
    param: 击打坐标的平均值, box_width, box_height
    return: 矩形框角点信息
    '''
    x, y = average_mid_point
    w, h = box_width / 2, box_height / 2
    point1 = [x + w, y + h]
    point2 = [x - w, y + h]
    point3 = [x - w, y - h]
    point4 = [x + w, y - h]
    box_info = [point1, point2, point3, point4]
    return box_info


def calAverageMidPoint(center_coordinates_list):
    '''
    算出击打坐标的平均值
    param: center_coordinates_list 传进来的坐标点
    return: 
    '''
    center_cooedinates_array = np.array(center_coordinates_list)
    x, y = np.mean(center_cooedinates_array, axis=0)
    average_midrect = (x, y)
    return average_midrect


def calBoxCenter(box: list) -> tuple:
    """
    计算矩形框中心
    :param box: 矩形四点坐标
    :return: 矩形中心点坐标
    """
    box = np.array(box)
    center = (sum(box[:, 0]) // 4, sum(box[:, 1]) // 4)
    return center


def calMomentCenter(contours_list: list) -> tuple:
    """
    计算轮廓的几何矩中心
    :param contours_list: 轮廓
    :return: 中心点
    """
    momentCenterPoint = None
    if len(contours_list) != 0:
        contour = max(contours_list, key=cv2.contourArea)

        mm = cv2.moments(contour)  # 求轮廓的几何矩
        cx = mm['m10'] / mm['m00']  # 原点的零阶矩
        cy = mm['m01'] / mm['m00']
        momentCenterPoint = (int(cx), int(cy))  # 几何矩中心
    return momentCenterPoint


def calLengthWidthRatio(box: list) -> float:
    """
    计算矩形框的长宽比
    :param box: [(x1,y1),(x2,y2),(x3,y3),(x4,y4)] 类型矩形框
    :return: 长宽比
    """
    ratio = None
    if len(box) > 0:
        len1 = calPointDistance(box[0], box[2])
        len2 = calPointDistance(box[0], box[1])
        height, weight = min([len1, len2]), max([len1, len2])
        if height != 0:
            ratio = float(weight) / float(height)  # 长宽比
    return ratio


def calPointDistance(point1: tuple, point2: tuple) -> float:
    """
    计算两像素点间的距离
    :param point1: 点 1
    :param point2: 点 2
    :return: 两点距离
    """
    distance = math.sqrt(math.pow((point1[0] - point2[0]), 2) + math.pow((point1[1] - point2[1]), 2))
    return distance


def calLineRadians(point1: tuple, point2: tuple) -> float:
    """
    以直线终点为原点绘制x轴并计算直线与x轴夹角大小(弧度制）
    :param point1: 直线起点
    :param point2: 直线终点
    :return: 直线与坐标轴夹角大小（弧度制）
    """
    h = point1[1] - point2[1]
    w = point1[0] - point2[0]

    radians = math.atan2(h, w)
    return radians

def calOrthogonalVelocity(point1: tuple, point2: tuple,time_gap:float):
    """计算直线正交分解后的线速度

    :param point1: 点 1
    :param point2: 点 2
    :param time_gap: 时间差
    :return: 延x轴和y轴方向的线速度
    """
    x_vel = (point1[0]-point2[0]) / time_gap
    y_vel = (point1[1]-point2[1]) / time_gap
    return x_vel,y_vel

def calAngularVelocity(last_radian: float, radian: float, time_differ: float):
    """计算角速度（弧度制）
    :param last_radian: 上一次的弧度
    :param radian: 当前弧度
    :param time_differ: 两数据之间的时间差
    :return: 角速度
    """
    radian_differ = abs(radian - last_radian)
    AngleVelo = radian_differ / time_differ
    return AngleVelo


def calDownTime(distance: float, gimbal_pitch: float, ball_speed: float):
    """
    运用迭代法计算弹丸下坠时间
    :param distance: 目标中心距相机坐标系中心的距离
    :param gimbal_pitch: 云台当前pitch角度（弧度制）从下位机获取
    :param ball_speed: 弹丸初速度 从下位机获取
    :return: 弹丸坠落时间
    """
    g = 9.778
    X = distance * math.cos(gimbal_pitch)  # 目标物体中心与相机坐标系中心水平方向上的距离差
    Y = distance * math.sin(gimbal_pitch)  # 目标物体中心与相机坐标系中心垂直方向上的距离差
    for i in range(20):
        down_time = distance / 1000.0 / ball_speed  # 下坠时间
        offset_gravity = 0.5 * g * down_time ** 2 * 1000  # 下落距离
        new_angle = math.atan((Y + offset_gravity) / X)
        distance = abs(X / math.cos(new_angle))
    return down_time


def findPeakTrough(vel_list):  # TODO rename
    peaks = signal.find_peaks(vel_list, distance=20)[0]
    negative_vel_list = np.multiply(np.array(vel_list), np.array(-1))
    troughs = signal.find_peaks(negative_vel_list, distance=20)[0]
    peak_index, troughs_index = peaks[0], troughs[0]

    peak = [peak_index, vel_list[peak_index]]  # vel_list[peak_index] 波峰的值
    trough = [troughs_index, vel_list[troughs_index]]
    return peak, trough


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
    max_len_point = [point_list[0], point_list[dis_list.index(sort_dis_list[1]) + 1]]
    return max_len_point


def rigidTransform(theta: float, tx: int, ty: int, rect_list: list) -> list:
    """
    对矩形框进行刚体变换
    :param theta: 刚体变换旋转角度（角度制）
    :param tx: 旋转中心 x坐标
    :param ty: 旋转中心 y坐标
    :param rect_list: 矩形四点坐标
    :return: 刚体变换后的矩形四点坐标
    """
    new_point_list = []
    rotation_matrix = cv2.getRotationMatrix2D((tx, ty), theta, 1)
    for point in rect_list:
        point = np.array([[point[0]], [point[1]], [1]])
        new_point = np.dot(rotation_matrix, point)
        new_point = np.rint(new_point)  # 四舍五入取整
        new_point_list.append(new_point.reshape(1, 2).tolist()[0])
    return new_point_list


def sortCoordinate(point_list: list) -> list:
    """
    对矩形框四点坐标进行排序，顺序为 左上，右上，右下，左下
    :param point_list: 矩形四点坐标
    :return: 顺时针排序的矩形四点坐标
    """
    max_len_point = getRectMaxLenPoint(point_list)
    radian = calLineRadians(max_len_point[0], max_len_point[1])
    degree = math.degrees(radian)
    if degree < 0:  degree += 180
    if degree > 90: degree -= 180
    center = calBoxCenter(point_list)
    rigid_point = np.array(rigidTransform(degree, int(center[0]), int(center[1]), point_list))
    x_min = np.min(rigid_point[:, 0])
    x_max = np.max(rigid_point[:, 0])
    y_min = np.min(rigid_point[:, 1])
    y_max = np.max(rigid_point[:, 1])

    new_point_list = [[x_min, y_min], [x_max, y_min], [x_max, y_max], [x_min, y_max]]
    new_rigid_point = rigidTransform(-degree, int(center[0]), int(center[1]), new_point_list)
    return new_rigid_point

def calDownTime(distance: float, gimbal_pitch: float, ball_speed: float):
    """
    运用迭代法计算弹丸下坠时间
    :param distance: 目标中心距相机坐标系中心的距离
    :param gimbal_pitch: 云台当前pitch角度（弧度制）从下位机获取
    :param ball_speed: 弹丸初速度 从下位机获取
    :return: 弹丸坠落时间
    """
    g = 9.778
    gimbal_pitch = -gimbal_pitch
    X = distance * math.cos(gimbal_pitch)  # 目标物体中心与相机坐标系中心水平方向上的距离差
    for i in range(20):
        Y = distance * math.sin(gimbal_pitch)  # 目标物体中心与相机坐标系中心垂直方向上的距离差
        down_time = distance / 1000.0 / ball_speed  # 下坠时间
        offset_gravity = 0.5 * g * down_time ** 2 * 1000  # 下落距离
        new_radian = math.atan((Y + offset_gravity) / X)
        distance = abs(X / math.cos(new_radian))
    return down_time

def adjustBallistics(pnp_info, gimbal_pitch, ball_speed):
    g = 9.778
    copied_pnp_info = deepcopy(pnp_info)
    gimbal_pitch = -gimbal_pitch
    ori_radian = gimbal_pitch + copied_pnp_info[2]
    
    ori_distance = copied_pnp_info[0]
    ori_Y = ori_distance * math.sin(ori_radian)
    ori_Z = ori_distance * math.cos(ori_radian)
    try:
        temp_distance = copied_pnp_info[0]
        temp_radian = gimbal_pitch + copied_pnp_info[2]
        for i in range(20):
            Y = temp_distance * math.sin(temp_radian)
            down_time = temp_distance / 1000.0 / ball_speed  # 下坠时间
            offset_gravity = 0.5 * g * down_time ** 2 * 1000  # 下落距离
            if abs(Y - offset_gravity - ori_Y) < 20:
                break
            temp_radian = math.atan((Y+offset_gravity)/ori_Z)
            temp_distance = abs(ori_Z/math.cos(temp_radian))
            adjust_radian = temp_radian-gimbal_pitch
        adjust_radian = temp_radian-gimbal_pitch - copied_pnp_info[2] 
        copied_pnp_info[2] = (copied_pnp_info[2]-adjust_radian)
    except ZeroDivisionError:
        pass
    return copied_pnp_info

def calIOU(box_rotation_info1, box_rotation_info2):
    """比较图像框交并补"""
    iou = 0
    area_ratio  = 0
    int_pts = cv2.rotatedRectangleIntersection(tuple(box_rotation_info1), tuple(box_rotation_info2))[1]
    if int_pts is not None:
        area1, area2 = box_rotation_info1[1][0] * box_rotation_info1[1][1], box_rotation_info2[1][0] * box_rotation_info2[1][1]
        order_pts = cv2.convexHull(int_pts, returnPoints=True)
        int_area = cv2.contourArea(order_pts)
        iou = int_area / (area1 + area2 - int_area)
        area_ratio = max(area1,area2)/min(area1,area2)
    if area_ratio > 2:
        iou = 0 
    return iou

class Point_Debuger(Node):
    def __init__(self):
        super().__init__('Bubble_Debuger')
        self.point_sub = self.create_subscription(
            BoundingPolygonBoxes2D, '/cv/armour', self.point_callback, 5)
        self.rviz_test_pub = self.create_publisher(
            PointStamped, '/debug/armour/point', 10)
        self.rviz_test_pose_pub = self.create_publisher(
            PoseStamped, '/debug/armour/pose', 10)
        

    def solve_info(self, box_points, target_type: str, target_num: str):
        if target_type.lower() == "armour":
            # 大装甲板
            if int(target_num) in [1, 8]:
                armour_size = armourSize["bigArmour"]
            # 小装甲板
            else:
                armour_size = armourSize["normalArmour"]
        elif target_type.lower() == "rune":
            # 能量机关装甲板
            armour_size = armourSize["energyArmour"]
        w, h = armour_size[0] / 2, armour_size[1] / 2

        world_coordinate = np.array([
            [-w, h, 0],
            [w, h, 0],
            [w, -h, 0],
            [-w, -h, 0]
        ], dtype=np.float64)
        # 像素坐标
        # pnts = np.array(box_points, dtype=np.float64)

        # rotation_vector 旋转向量 translation_vector 平移向量
        pnts = np.array(sortCoordinate(box_points), dtype=np.float64)
        # print("aaa",pnts)
        success, rvec, tvec = cv2.solvePnP(
            world_coordinate, pnts, Camera_intrinsic["mtx"], Camera_intrinsic["dist"])
        distance = np.linalg.norm(tvec)
        print(distance)
        yaw_angle, pitch_angle = np.arctan(
            tvec[0] / tvec[2]), np.arctan(tvec[1] / tvec[2])
        return tvec, rvec

    def point_callback(self, data: BoundingBoxes2D):
        time_stamp = data.image_header.stamp
        for box in data.bounding_boxes:
            box_points = [[int(pose.x), int(pose.y)]
                          for pose in box.pose.points]
            # print(box_points)
            if len(box_points) == 0:
                return

            # box_rect = cv2.minAreaRect(np.array(box_points))
            # box_rect = cv2.boxPoints(box_rect)

            target_type = box.class_id
            target_num = box.type
            obj_tvec, obj_rvec = self.solve_info(box_points, target_type, target_num)
            # obj_tvec, obj_rvec = self.solve_info(box_rect, target_type, target_num)
            distance = np.linalg.norm(obj_tvec)
            # print(distance)

            if obj_tvec[2] <= 0 or not (0.05 <= distance <= 6):
                return

            msg = PointStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = float(obj_tvec[0])
            msg.point.y = float(obj_tvec[1])
            msg.point.z = float(obj_tvec[2])
            self.rviz_test_pub.publish(msg)

            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(obj_tvec[0])
            msg.pose.position.y = float(obj_tvec[1])
            msg.pose.position.z = float(obj_tvec[2])
            dst, jac = cv2.Rodrigues(obj_rvec)
            msg.pose.orientation.w = 0.5 * \
                (1+dst[0][0]+dst[1][1]+dst[2][2])**(0.5)
            msg.pose.orientation.x = (
                dst[2][1]-dst[1][2]) / (4*msg.pose.orientation.w)
            msg.pose.orientation.y = (
                dst[0][2]-dst[2][0]) / (4*msg.pose.orientation.w)
            msg.pose.orientation.z = (
                dst[1][0]-dst[0][1]) / (4*msg.pose.orientation.w)
            self.rviz_test_pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    debuger = Point_Debuger()
    rclpy.spin(debuger)
    debuger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
