'''
Author: HarryWen
Date: 2022-08-06 23:07:53
FilePath: /bubble_aiming/bubble_aiming/predictor/outpostPredictor.py
LastEditors: HarryWen
LastEditTime: 2022-08-07 21:41:49
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import queue
from bubble_aiming.decision.armourDecision import *
from bubble_aiming.decision.runeDecision import *
from bubble_aiming.utils.Calculator import *
from bubble_aiming.utils.Comparator import *
from bubble_aiming.utils.Converter import *
from bubble_aiming.utils.DataType import *
from bubble_aiming.compensator.compensator import *

MAX_HISTORY_SIZE = 200


class OutpostPredictor:
    def __init__(self, image_size) -> None:
        # self.shoot_queue = FIFO_Queue(MAX_HISTORY_SIZE)  # 判断是否到达发射时间的队列
        # self.t_queue = FIFO_Queue(2)  # 动态更迭周期时长的队列

        self.time_queue = FIFO_Queue(MAX_HISTORY_SIZE)  # 判断是否到达发射时间的队列
        self.respond_time = 0  # 云台相应(响应）时间
        self.is_shoot = False 
        self.image_size = image_size

        self.last_reach_center_t = 0
        self.count = 0
        self.period_queue = FIFO_Queue(10,[0])
        self.standard = Position(0,0)
        self.ballisticCompensator = BallisticCompensator()
        self.bullet_vel = 0

    def predict(self, target, period, bullet_vel):
        # 计算是否满足预期发射时间间隔
        self.bullet_vel = bullet_vel
        self.time_queue.append(target.get_stamp())
        shoot_gap = self.calShootGap(target, bullet_vel, period) # 计算射击间隔
        if abs(self.time_queue[-1] - self.time_queue[0] - shoot_gap) <= 0.1:
            print("开炮!")
            if not self.is_shoot: self.is_shoot = True
            self.time_queue.clear()
        # TODO 后续return发射信号

    def calShootGap(self, target, bullet_vel, period):
        shoot_gap = period
        if not self.is_shoot:
            down_time = self.calDownTime(target, bullet_vel)  # 下坠时间计算可能有问题
            shoot_gap = period - (down_time + self.respond_time)  # 首次周期t_first = （补偿+响应+距离)
            # print(shoot_gap)
        return shoot_gap
            
    def updatePeriod(self, target):
        # 更迭周期的函数
        if self.reachCenter(target):
            if self.last_reach_center_t == 0:
                self.last_reach_center_t = target.stamp
            else:
                # 防止产生由于距离过渡值导致的同一块装甲板重复识别
                if compareTimeGap(self.last_reach_center_t, target.stamp, 0.1):
                    period = abs(self.last_reach_center_t - target.stamp)
                    self.period_queue.append(period)
                    # self.period_queue.append(abs(self.last_reach_center_t - target.stamp))
                    self.last_reach_center_t = target.stamp
        period = np.mean(self.period_queue[:])
        return period


    def reachCenter(self, target):
        # 判断装甲板是否经过瞄点
        judgement = False
        tar_center = target.get_rect_rotation()[RotationInfo.center]
        if abs(tar_center[0] - self.image_size[0]/2) < 10:
            judgement = True
        return judgement


    # def recData(self, data_queue: queue, time_stamp: float) -> None:
    #     # 队列数据添加
    #     data_queue.append(time_stamp)

    # def calComTime(self, respond_time, period, down_time):
        # # 计算补偿时间
        # compensation_time = -1
        # for i in range(10):
        #     compensation_time = i * period - respond_time - down_time
        #     # 补偿时间+距离时间+响应时间 = 整数倍 * period
        #     if compensation_time >= 0:
        #         break
        # return compensation_time

    def calDownTime(self, target, bullet_vel):
        # 计算下落时间
        g = 9.778
        p = 1.169  # kg/m^3 弹丸的密度
        C = 0.47  # 待测的空气粘滞度
        S = 1418.63 * 1e-6  # mm^2 -> m^2 弹丸的空阻接触面积
        M = 41 * 1e-3  # g -> kg 弹丸的质量
        k0 = C * p * S / 2
        ori_bullet_vel = bullet_vel  # 储存弹丸初速作为v0
        try:
            for _ in range(20):
                down_time = target.distance / bullet_vel  # 下坠时间
                bullet_vel = M / (k0 * down_time + M / ori_bullet_vel)  # 速度迭代
        except ZeroDivisionError:
            pass
        return down_time

    def targetSift(self, target, present):
        if self.count == 0:
            if self.reachCenter(target) != "True":
                target = self.ballisticCompensator.adjustBallistics(target,present.pitch_angle, self.bullet_vel)
                print(target.pitch_angle)
                self.standard = Position(present.pitch_angle + target.pitch_angle, present.yaw_angle)
                # 储存第一次补偿后的位置信息为standard
                self.count += 1
                # 跳出首次发射计算
            else:
                target.pitch_angle = target.yaw_angle = 0
        else:
            target.pitch_angle = self.standard.pitch_angle - present.pitch_angle
            print("target.pitch_angle =",target.pitch_angle)
            target.yaw_angle = self.standard.yaw_angle - present.yaw_angle
            print("target.yaw_angle = ",target.yaw_angle)
        return target
