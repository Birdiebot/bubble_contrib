'''
Author: Ligcox
Date: 2022-02-07 05:03:19
FilePath: /bubble/src/bubble_contrib/bubble_aiming/bubble_aiming/utils/Converter.py
LastEditors: HarryWen
LastEditTime: 2022-08-20 19:39:35
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science.
'''

import math

import cv2
import numpy as np

from bubble_aiming.utils.Calculator import *
from numpy import tan, arctan as atan, sin, cos, arcsin as asin, pi

armourSize = {
    "bigArmour": [0.235, 0.058],
    "normalArmour": [0.140, 0.058],
    "energyArmour": [0.230, 0.127]
}



def cvBoxPoints2pose(cvdata: cv2.boxPoints) -> list:
    (x0, y0), (x1, y1) = cvdata
    for i in [x0, y0, x1, y1]:
        i = float(i)
    w = x1 - x0
    h = y1 - y0
    return [(x0, y0), (x0+w, y1), (x1, y1), (x0+h, y1)]


def cvtToRectPNP(target, offset, camera_intrinsic):
    
    """数据格式转换，转换至目标位姿信息"""
    if target.name == "Armour":
        if int(target.target_type) in [1, 8]:
            armour_size = armourSize["bigArmour"]
        else:
            armour_size = armourSize["normalArmour"]
    else:
        armour_size = armourSize["energyArmour"]

    w, h = armour_size[0] / 2, armour_size[1] / 2
    world_coordinate = np.array(
        [
            [-w, h, 0],
            [w, h, 0],
            [w, -h, 0],
            [-w, -h, 0]
        ], dtype=np.float64
    )

    # pixel coordinate
    pnts = np.array(sortCoordinate(target.get_box_points()), dtype=np.float64)

    success, rvec, tvec = cv2.solvePnP(
        world_coordinate, pnts, camera_intrinsic["mtx"], camera_intrinsic["dist"])
    # convert data pose from camera_optical to gimbal coordinate
    tvec[0] += offset["x_offset"]
    tvec[1] += offset["y_offset"]
    tvec[2] += offset["z_offset"]

    return rvec, tvec


def cvtToRectPoint(rect_rotation_info):
    """数据格式转换，将旋转矩形转换至角点矩形"""
    rect_points_info = np.int0(cv2.boxPoints(
        tuple(rect_rotation_info))).tolist()
    return rect_points_info


def cvtToRectRotation(rect_point_info):
    """数据格式转换，将角点矩形转换至旋转矩形"""
    rect_rotation_info = cv2.minAreaRect(
        np.array(rect_point_info, dtype="float32"))
    return rect_rotation_info


def cvtToVecRodrigue(tvec):
    distance = np.linalg.norm(tvec)
    yaw_angle, pitch_angle = np.arctan(
        tvec[0] / tvec[2]), np.arctan(tvec[1] / tvec[2])
    rect_pnp_info = [distance, float(yaw_angle), float(pitch_angle)]
    rect_rotation_info, _ = cv2.Rodrigues(rvec)


def cvtPixelToRadian(point: List[int] = [0, 0], res: List[int] = [0, 0], fov: List[float] = [0, 0], camera_roll: float = 0,
                     inverse_y: bool = False):
    """
    根据相机fov(视场角)，将图像中像素点相对于相机坐标系中心的弧度，转换为世界坐标系下的弧度
    :param point: 图像中的像素点
    :param res: 图像大小
    :param fov: 相机视场角
    :param camera_roll: 相机旋转角度
    :param inverse_y: 是否将图像绕y轴翻转
    """
    [hw, hh] = [v * 0.5 for v in res]
    lp = pi * 0.9  # limit for fov
    [hy, hp] = [min(lp, max(v, -lp)) * 0.5 for v in fov]
    off_x = point[0] - hw
    off_y = point[1] - hh
    rad_y = atan(off_x * tan(hy) / hw)
    rad_p = atan(off_y * tan(hp) / hh)
    sin_y = sin(rad_y)
    sin_p = sin(rad_p)
    if inverse_y:
        fixed_y = asin(sin_y * cos(camera_roll) + sin_p * sin(camera_roll))
        fixed_p = asin(- sin_y * sin(camera_roll) + sin_p * cos(camera_roll))
    else:
        fixed_y = asin(sin_y * cos(camera_roll) - sin_p * sin(camera_roll))
        fixed_p = asin(sin_y * sin(camera_roll) + sin_p * cos(camera_roll))
    return [fixed_p, fixed_y]
