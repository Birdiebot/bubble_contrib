"""
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
License: GNU General Public License v3.0.
See LICENSE file in root directory.

Author: your name your email
Date: 2022-08-18 17:27:59
FilePath: /bubble_bringup/home/nvidia/Desktop/Projects/PythonProjects/bubble/src/bubble_contrib/bubble_aiming/bubble_aiming/compensator/compensator.py
LastEditors: your name your email
LastEditTime: 2022-09-14 13:43:04
"""

import math
from copy import deepcopy
from scipy.spatial.transform import Rotation
import numpy as np
from bubble_aiming.utils.DataType import *
from bubble_aiming.utils.Filter import *
GRAVITY = 9.778


class BallisticCompensator():
    def __init__(self) -> None:
        self.g = GRAVITY
        self.dis_list = FIFO_Queue(100)
        self.last_pitch_angle = 0
        self.last_distance = 0

    def recDistance(self, distance):
        is_record = True
        round_distance = np.round(distance, 1)

        if not math.isnan(round_distance):
            self.dis_list.append(round_distance)

        return is_record

    def adjustBallistics(self, target, gimbal_pitch, ball_speed):
        copied_target = deepcopy(target)
        gimbal_pitch = -gimbal_pitch
        ori_distance = target.distance
        self.recDistance(ori_distance)
        try:
            if self.dis_list.size() > 30:
                ori_distance = lowPassFilter(3, 0.05, self.dis_list[:])[-5]
                ori_radian = gimbal_pitch + copied_target.pitch_angle  # 目标相对云台的pitch偏转角度
                ori_Y = ori_distance * math.sin(ori_radian)  # 目标距离相机坐标系中心的竖直距离
                ori_Z = ori_distance * math.cos(ori_radian)  # 目标距离相机坐标系中心的水平距离
                iter_dis = ori_distance
                iter_radian = ori_radian
                for _ in range(20):
                    Y = iter_dis * math.sin(iter_radian)
                    down_time = iter_dis / ball_speed  # 下坠时间
                    offset_gravity = 0.5 * self.g * down_time ** 2  # 下落距离
                    error_value = abs(Y - offset_gravity - ori_Y)
                    iter_radian = math.atan((Y+error_value)/ori_Z)
                    iter_dis = abs(ori_Z/math.cos(iter_radian))
                adjust_radian = iter_radian - gimbal_pitch - copied_target.pitch_angle
                copied_target.pitch_angle = copied_target.pitch_angle - adjust_radian
                self.last_pitch_angle = copied_target.pitch_angle
            else:
                copied_target.pitch_angle = self.last_pitch_angle

        except ZeroDivisionError:
            pass

        return copied_target

    def calDownTime(self, target, gimbal_pitch: float, ball_speed: float):
        """
        运用迭代法计算弹丸下坠时间
        :param distance: 目标中心距相机坐标系中心的距离
        :param gimbal_pitch: 云台当前pitch角度(弧度制）从下位机获取
        :param ball_speed: 弹丸初速度 从下位机获取
        :return: 弹丸坠落时间
        """
        g = GRAVITY
        gimbal_pitch = -gimbal_pitch
        distance = target.distance
        X = distance * math.cos(gimbal_pitch)  # 目标物体中心与相机坐标系中心水平方向上的距离差
        for i in range(20):
            Y = distance * math.sin(gimbal_pitch)  # 目标物体中心与相机坐标系中心垂直方向上的距离差
            down_time = distance / 1000.0 / ball_speed  # 下坠时间
            offset_gravity = 0.5 * g * down_time ** 2 * 1000  # 下落距离
            new_radian = math.atan((Y + offset_gravity) / X)
            distance = abs(X / math.cos(new_radian))
        return down_time


def adjustMotionPosition(trans: np.array, target: ArmourInfo, gimbal: GimbalInfo, chissise_vel, bullet_vel):
    if trans is not None:
        trans_vec = coord2TransVec(trans)
        # convert 3D position to homogeneous coordinates
        # tran_pose = np.dot(trans_vec,
        #                    np.row_stack((target.get_position_xyz(), [1])))
        tran_pose = np.dot(np.linalg.inv(trans_vec),
                           np.row_stack((target.get_position_xyz(), [1])))
        angle = motion_angle(tran_pose, chissise_vel, bullet_vel)
        yaw_angle = angle[0] - gimbal.get_yaw()
        pitch_angle = -angle[1] - gimbal.get_pitch()
        target.set_rotation_rpy(yaw_angle, pitch_angle, 0)
        # print("final ", np.degrees(target.yaw_angle),
        #       np.degrees(target.pitch_angle))
    return target


def motion_angle(p: list, chassis_vel: np.array, bullet_vel: float = 30) -> tuple:
    g = GRAVITY
    px, py, pz = p[0], p[1], p[2]
    # print("pxpypz", px, py, pz)
    chassis_vel = np.linalg.norm(chassis_vel)
    OP_abs = np.linalg.norm(p[:3])
    t = np.poly1d([
        1/4*g**2,
        0,
        chassis_vel**2 + g*pz - bullet_vel**2,
        -2*py*chassis_vel,
        OP_abs**2
    ])

    print("****FUNC****")
    print(t)
    print(t.r)
    t = min(t.r[t.r > 0])
    # sin_yaw = (py-chassis_vel*t) / (bullet_vel*t)
    # sin_pitch = (-pz-0.5*g*t**2) / (bullet_vel*t)
    # pitch_angle = np.arcsin(sin_pitch)
    # cos_pitch = np.cos(pitch_angle)
    # sin_yaw = (py-chassis_vel*t) / (bullet_vel*t*cos_pitch)
    # yaw_angle = np.arcsin(sin_yaw)
    pitch_angle = np.math.atan2(
        -pz-0.5*g*t**2, (px**2+(py-chassis_vel*t)**2)**0.5)
    yaw_angle = np.math.atan2(py-chassis_vel*t, px)

    # print("yaw_angle", yaw_angle, "pitch_angle", pitch_angle)

    print("compensation", np.degrees(yaw_angle), np.degrees(pitch_angle))

    # print("=======================")
    # print("t", t, "bullet_vel*t", bullet_vel*t)
    # print(f"px:{px},   {bullet_vel*t*np.math.cos(pitch_angle)*np.math.cos(yaw_angle)}")
    # print(f"py:{py},   {bullet_vel*t*np.math.cos(pitch_angle)*np.math.sin(yaw_angle)+chassis_vel*t}")
    # print(f"pz:{pz},   {-bullet_vel*t*np.math.sin(pitch_angle)-0.5*g*t**2}")

    # print("=======================")
    return yaw_angle, pitch_angle


def coord2TransVec(trans):
    rotation = trans.transform.rotation
    translation = trans.transform.translation
    rotation_matrix = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
    # t = np.array([[translation.x], [translation.y], [translation.z]])
    t = np.array([[0], [0], [0]])
    T = np.column_stack((rotation_matrix, t))
    T = np.row_stack((T, [0, 0, 0, 1]))
    return T


def adjustBallistics_try(target: ArmourInfo, gimbal_pitch, ball_speed) -> ArmourInfo:
    g = GRAVITY
    p = 1.169          # kg/m^3
    C = 0.47           # 待测
    S = 1418.63 * 1e-6  # mm^2 -> m^2
    M = 41 * 1e-3      # g -> kg
    gimbal_pitch = -gimbal_pitch
    ori_distance = target.distance
    k0 = C*p*S / 2
    k1 = k0 / M
    ori_ball_speed = ball_speed  # 储存弹丸初速作为v0
    copied_target = deepcopy(target)
    ori_radian = gimbal_pitch + copied_target.pitch_angle
    ori_Y = ori_distance * math.sin(ori_radian)
    ori_Z = ori_distance * math.cos(ori_radian)
    try:
        temp_distance = copied_target.distance
        temp_radian = gimbal_pitch + copied_target.pitch_angle
        Z = temp_distance * \
            math.cos(copied_target.pitch_angle) * \
            math.cos(copied_target.yaw_angle)
        print(temp_distance)
        if temp_distance > 5.0:
            for _ in range(50):
                Y = temp_distance * math.sin(temp_radian)
                time = Z / ball_speed  # 下坠时间
                # print("time",time)
                offset_gravity = 0.5 * g * time ** 2  # 下落距离
                ball_speed = M / (k0*time + M/ori_ball_speed)
                if abs(Y - offset_gravity - ori_Y) < 0.4:
                    break
                temp_radian = math.atan((Y+offset_gravity)/ori_Z)
                Z = abs(ori_Z/math.cos(temp_radian))
                adjust_radian = temp_radian-gimbal_pitch
        else:
            for _ in range(50):
                Y = temp_distance * math.sin(temp_radian)
                time = Z / ball_speed  # 下坠时间
                # print("time",time)
                offset_gravity = 0.5 * g * time ** 2  # 下落距离
                ball_speed = M / (k0*time + M/ori_ball_speed)
                if abs(Y - offset_gravity - ori_Y) < 0.2:
                    break
                temp_radian = math.atan((Y+offset_gravity)/ori_Z)
                Z = abs(ori_Z/math.cos(temp_radian))
                adjust_radian = temp_radian-gimbal_pitch
        adjust_radian = temp_radian-gimbal_pitch - copied_target.pitch_angle
        copied_target.pitch_angle = (copied_target.pitch_angle - adjust_radian)

    except ZeroDivisionError:
        pass

    return copied_target
