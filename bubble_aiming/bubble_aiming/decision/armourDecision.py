'''
Author: HarryWen
Date: 2022-07-17 03:27:36
FilePath: /bubble/src/bubble_contrib/bubble_aiming/bubble_aiming/decision/armourDecision.py
LastEditors: HarryWen
LastEditTime: 2022-07-17 18:56:04
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import numpy as np

from bubble_aiming.utils.Calculator import *
from bubble_aiming.utils.DataType import *
from bubble_aiming.utils.Filter import *

RMUC_Robot_Type = {1: "hero", 2: "engineer", 3: "infantry", 4: "infantry", 5: "infantry", 7: "sentry",
                   8: "outpost", 9: "base"}

RMUA_Robot_Type = {1: "infantry_1", 2: "infantry_2"}

rmuc_ground_robot_num = [1, 2, 3, 4, 5]
rmuc_construction_num = [7, 8, 9]

rmua_ground_robot_num = [1, 2]

construction_priority = 0.4
ground_priority = 1 - construction_priority


#  比较目标距离，距离近的优先级高
def compareDistance(distance_array: np.ndarray) -> np.ndarray:
    sort_distance_array = np.sort(distance_array)[::-1]
    compared_distance = np.array([np.array(np.where(sort_distance_array == i)).min(
    ) for i in distance_array]).flatten()  # 加一保证排序从1开始
    norm_distance = np.around(compared_distance / np.sum(compared_distance), 1)
    return compared_distance


def compareAngle(rect_rotate_info: np.ndarray) -> np.ndarray:
    """
    此处可根据情况是否需要根据pitch和yaw的角度来比较优先级
    """
    yaw_array, pitch_array, raw_array = rect_rotate_info[:,
                                                         0], rect_rotate_info[:, 1], rect_rotate_info[:, 2]
    sort_yaw_array = np.sort(yaw_array)[::-1]
    compared_angle = np.array([np.where(sort_yaw_array == i)
                               for i in yaw_array]).flatten()  # 加一保证排序从1开始
    norm_angle = np.around(compared_angle / np.sum(compared_angle), 1)
    return norm_angle


def compareTargetCenter(target_center_list: list, img_size: tuple) -> int:
    """
    Compares the distance between the target center and the image center
    """
    img_width, img_height = img_size
    rect_center_array = np.array(target_center_list)
    x_gap = abs(rect_center_array[:, 0]-(img_width/2))
    y_gap = abs(rect_center_array[:, 1]-(img_height/2))
    tar_index = x_gap.argmin()
    return tar_index


def compareHp(hp_array: np.ndarray) -> np.ndarray:
    sort_hp_array = np.sort(hp_array)[::-1]
    compared_hp_array = np.array(
        [np.where(sort_hp_array == i) for i in hp_array]).flatten()  # 加一保证排序从1开始
    norm_hp_array = np.around(compared_hp_array / np.sum(compared_hp_array), 1)
    return norm_hp_array


def compareTar(tar_type_array: np.ndarray, game_type: int, priority: int) -> np.ndarray:
    # 机甲大师赛
    if game_type == 1:
        ground_robot = np.intersect1d(rmuc_ground_robot_num, tar_type_array)
        construction = np.intersect1d(rmuc_construction_num, tar_type_array)
        construction_len, ground_robot_len = len(
            construction), len(ground_robot)
        ground_robot_ratio = np.repeat(
            (np.array(ground_priority / ground_robot_len), 3), ground_robot_len)
        construction_ratio = np.repeat(
            (np.array(construction_priority / construction_len), 3), construction_len)
        if priority in ground_robot:
            ground_robot_ratio = np.repeat(
                (ground_priority - ground_priority * 0.5) / ground_robot_len, ground_robot_len)
            ground_robot_ratio[np.where(
                ground_robot == priority)] += ground_priority * 0.5
            ground_robot_ratio = np.around(ground_robot_ratio, decimals=1)
        elif priority in construction:
            construction_ratio = np.repeat(
                (construction_priority - construction_priority * 0.5) / construction_len, construction_len)
            construction_ratio[np.where(
                construction == priority)] += construction_priority * 0.5
            construction_ratio = np.around(construction_ratio, decimals=1)

        return np.hstack((ground_robot_ratio, construction_ratio))
    # 机甲大师单项赛
    elif game_type == 2:
        pass
    # 人工智能挑战赛
    elif game_type == 3:
        ground_robot = np.intersect1d(rmua_ground_robot_num, tar_type_array)
        ground_robot_len = len(ground_robot)
        ground_robot_ratio = np.repeat(
            (np.array(ground_priority / ground_robot_len), 3), ground_robot_len)

        if priority in ground_robot:
            ground_robot_ratio = np.repeat(
                (ground_priority - ground_priority * 0.5) / ground_robot_len, ground_robot_len)
            ground_robot_ratio[np.where(
                ground_robot == priority)] += ground_priority * 0.5
            ground_robot_ratio = np.around(
                ground_robot_ratio, decimals=1).tolist()

        return ground_robot_ratio
    # 联盟赛3V3
    elif game_type == 4:
        pass
    # 联盟赛1V1
    elif game_type == 5:
        pass

# 自瞄模式判断


class ArmourDecision:
    def __init__(self):
        self._gyro_factor = 0
        self._tolerance_factor = 0
        self._armour_alter_factor = 0
        self._armour_param = None
        "***********************************************************************"
        self._height_differ_min_thres = 0
        self._height_differ_max_thres = 0
        self._width_differ_min_thres = 0
        self._width_differ_max_thres = 0
        self._width_alter_min_thres = 0
        self._gyro_thres = 0
        self._tolerance_thres = 0
        self._armour_alter_thres = 0
        self._moving_alter_thres = 0
        self._gyro_iou_thres = 0
        "***********************************************************************"

    def setDebugParam(self, armour_param):
        # TODO 传入参数检查
        self._armour_param = armour_param

    def getDebugParam(self, *args):
        #  TODO 检查
        debug_param = self._armour_param
        for arg in args:
            param = debug_param[arg]
        return param

    def setArmour(self, armour):
        self._armour = armour

    def getArmour(self):
        first_armour, last_armour = self._armour[0], self._armour[-1]
        return first_armour, last_armour

    def parseArmourParam(self):
        self._height_differ_min_thres = self.getDebugParam(
            "height_differ_min_thres")  # TODO 改名 原：height_differ_minthreshold
        self._height_differ_max_thres = self.getDebugParam(
            "height_differ_max_thres")
        self._width_differ_min_thres = self.getDebugParam(
            "width_differ_min_thres")
        self._width_differ_max_thres = self.getDebugParam(
            "width_differ_max_thres")
        self._width_alter_min_thres = self.getDebugParam(
            "width_alter_min_thres")
        self._gyro_thres = self.getDebugParam("gyro_thres")
        self._tolerance_thres = self.getDebugParam("tolerance_thres")
        self._armour_alter_thres = self.getDebugParam("armour_alter_thres")
        self._moving_alter_thres = self.getDebugParam("moving_alter_thres")
        self._gyro_iou_thres = self.getDebugParam("gyro_iou_thres")

    def judgeMode(self):
        """
        根据装甲板信息进行对系数的处理
        param armour_list: [((x, y), (w, h), 0), .......]
        """
        mode = "aiming"
        self.parseArmourParam()  # 对传入参数进行解析
        first_armour, last_armour = self.getArmour()
        # gyro_debug_info = self.armour_param["gyro_debug_info"]
        first_armour_rotaion_info = first_armour.getRectRotationInfo(
            0, to_list=True)
        last_armour_rotaion_info = last_armour.getRectRotationInfo(
            0, to_list=True)
        if compareIOU(first_armour_rotaion_info, last_armour_rotaion_info, self._gyro_iou_thres):
            if compareHeight(self._height_differ_min_thres, self._height_differ_max_thres, first_armour_rotaion_info, last_armour_rotaion_info):
                if compareWidth(self._width_differ_min_thres, self._width_differ_max_thres, first_armour_rotaion_info, last_armour_rotaion_info):
                    # 判断是否为快速运动，慢速的小陀螺不启动反陀螺打击
                    self._gyro_factor += 1
                    # self.get_logger().info("self._gyro_factor += 1")
                else:
                    self._tolerance_factor += 1
                    # self.get_logger().info("_tolerance_factor: %d " % self._tolerance_factor)
            else:
                # 关闭反陀螺打击
                self.clearFactor()
                mode = "aiming"
        else:
            if compareHeight(self._height_differ_min_thres, self._height_differ_max_thres, first_armour_rotaion_info, last_armour_rotaion_info):
                if compareWidth(self._width_alter_min_thres, self._width_differ_max_thres, first_armour_rotaion_info, last_armour_rotaion_info):
                    # 装甲板发生切换，记录装甲板的数据
                    self._armour_alter_factor += 1
                    # alter_armour = True
                    # self.get_logger().info("alter_armour = True")
                else:
                    # 前后两帧装甲板的宽度差过大或过小
                    self._tolerance_factor += 1
                    # self.get_logger().info(" %d " % self._tolerance_factor)
            else:
                # 关闭反陀螺打击
                self.clearFactor()
                mode = "aiming"

        if self._gyro_factor >= self._gyro_thres:
            if self._armour_alter_factor >= self._armour_alter_thres:
                mode = "gyro"
            else:
                if self._tolerance_factor > self._tolerance_thres:
                    # 容忍限内依旧没有达到陀螺条件，默认不开启反陀螺打击
                    self.clearFactor()
                    mode = "aiming"
        else:
            if self._armour_alter_factor > self._moving_alter_thres and self._gyro_factor >= 1:
                # 陀螺+移动
                mode = "gyro"
            if self._tolerance_factor > self._tolerance_thres:
                # 容忍限内依旧没有达到陀螺条件，默认不开启反陀螺打击
                self.clearFactor()
                mode = "aiming"

        return mode

    def judgeArmourAlteration(self):
        """
        针对小陀螺判断装甲板是否存在切换
        @return:
        """
        alter_armour = False
        first_armour, last_armour = self.getArmour()
        first_armour_rotaion_info = first_armour.getRectRotationInfo(
            0, to_list=True)
        last_armour_rotaion_info = last_armour.getRectRotationInfo(
            0, to_list=True)
        if not compareIOU(first_armour_rotaion_info, last_armour_rotaion_info, self._gyro_iou_thres):
            if compareHeight(self._height_differ_min_thres, self._height_differ_max_thres, first_armour_rotaion_info, last_armour_rotaion_info):
                if compareWidth(self._width_alter_min_thres, self._width_differ_max_thres, first_armour_rotaion_info, last_armour_rotaion_info):
                    alter_armour = True
        return alter_armour

    def clearFactor(self):
        """系数清零"""
        self._tolerance_factor = 0
        self._gyro_factor = 0
        self._armour_alter_factor = 0
