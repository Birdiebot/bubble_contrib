'''
Author: HarryWen
Date: 2022-07-17 05:25:30
FilePath: /bubble_bringup/home/nvidia/Desktop/bubble/src/bubble_contrib/bubble_aiming/bubble_aiming/decision/runeDecision.py
LastEditors: HarryWen
LastEditTime: 2022-08-11 02:00:12
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import copy
import numpy as np
from enum import IntEnum

from bubble_aiming.utils.Filter import *
from bubble_aiming.utils.DataType import *
from bubble_aiming.utils.Calculator import *

# 能量机关模式判断
class RuneDecision:
    def __init__(self):
        self._mode = "Small_Power_Rune"
        self._wise = Wise.clockwise

    def judgeMode(self, stage_remain_time):
        """
        判断当前模式 使用比赛剩余时间进行判断
        """
        # 裁判系统读取剩余比赛时间
        if stage_remain_time < 200:  # 正常为180秒，200是为了提前去等待
            self._mode = "Large_Power_Rune"
        else:
            self._mode = "Small_Power_Rune"
        return self._mode

    def judgeDirectionRotation(self, degree_list):
        """
        判断能量机关旋转方向 使用间隔数据相减实现判断
        """
        interpolation_num = 2
        if len(degree_list) != 0:
            data_array = np.array(degree_list)
            data_array[data_array < 0] += 360
            dup_degree_list = copy.deepcopy(data_array.tolist())
            for i in range(interpolation_num):
                dup_degree_list.insert(0, 0)
            dup_degree_array = np.array(dup_degree_list)
            sub = np.subtract(
                data_array, dup_degree_array[:-interpolation_num])
            ratio = np.count_nonzero(sub > 0) / len(degree_list)
            if ratio < 0.5:
                self._wise = Wise.anticlockwise
            #  顺时针
            elif ratio >= 0.5:
                self._wise = Wise.clockwise  

        return self._wise
