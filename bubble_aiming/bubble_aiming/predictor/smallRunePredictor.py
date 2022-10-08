'''
Author: HarryWen
Date: 2022-07-16 15:48:48
FilePath: /bubble/src/bubble_contrib/bubble_aiming/bubble_aiming/predictor/smallRunePredictor.py
LastEditors: HarryWen
LastEditTime: 2022-07-26 18:06:44
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
from bubble_aiming.utils.Filter import *
from bubble_aiming.utils.DataType import *
from bubble_aiming.utils.Calculator import *


class SmallRunePredictor():
    def __init__(self):
        self.wise = Wise.clockwise

    def predict(self, rune_info, wise, predict_degree):
        self.wise = wise
        target = rune_info[-1][0]
        circle_center = target.get_center_bbox_rotation()[
            RotationInfo.center]
        # print(circle_center)
        tar_center = target.get_tar_bbox_rotation()[RotationInfo.center]
        rect_rotation_info = list(cvtToRectRotation(rigidTransform(
            predict_degree, tar_center, target.get_tar_bbox_points())))

        predict_center = self.calPredictedTarCenter(
            circle_center, tar_center, predict_degree)
        predict_rect = self.calPredictedTarRect(
            rect_rotation_info, predict_center, None)
        return predict_rect

    def calPredictedTarCenter(self, circle_center, tar_center, predict_degree):
        """
        计算预测矩形框的中心
        :param circle_center: 圆心
        :param predict_degree: 目标预测角度
        :return: 预测中心点坐标
        """
        radius = calPointDistance(circle_center, tar_center)  # 目标中心与旋转中心的距离
        if self.wise == Wise.anticlockwise:
            predict_degree = predict_degree
        elif self.wise == Wise.clockwise:
            predict_degree = -predict_degree

        predicted_tar_center = rigidTransform(predict_degree, circle_center, [tar_center])[0]
        return predicted_tar_center

    @staticmethod
    def calPredictedTarRect(rect_rotation_info, pred_center=None, pred_angle=None):
        """
        计算预测矩形的旋转框信息
        @param rect_rotation_info: 原始矩形框信息
        @param pred_center: 预测矩形框中心
        @param pred_angle: 预测矩形框旋转角度
        @return: 预测矩形旋转框信息
        """
        pred_tar_rect = []
        enum_info = RotationInfo
        if isinstance(rect_rotation_info, list):
            ori_width, ori_height = rect_rotation_info[enum_info.size]
            if pred_center is None:
                pred_center = rect_rotation_info[enum_info.center]
            if pred_angle is None:
                pred_angle = rect_rotation_info[enum_info.angle]
            pred_tar_rect = (pred_center, (ori_width, ori_height), pred_angle)
        return pred_tar_rect
