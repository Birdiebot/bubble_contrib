import copy
import math

import cv2
import numpy as np


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


RMUC_Robot_Type = {1: "hero", 2: "engineer", 3: "infantry", 4: "infantry", 5: "infantry", 7: "sentry",
                   8: "outpost", 9: "base"}

RMUA_Robot_Type = {1: "infantry_1", 2: "infantry_2"}

rmuc_ground_robot_num = [1, 2, 3, 4, 5]
rmuc_construction_num = [7, 8, 9]

rmua_ground_robot_num = [1, 2]

construction_priority = 0.4
ground_priority = 1 - construction_priority


# def commonComparator(value1,value2,thres):
#     state = False
#     if abs(value1-value2) > thres:
#         state = True
#     return state

def compareIoU(box_rotation_info1, box_rotation_info2, threshold_iou=0.5):
    """比较图像框交并补"""
    iou = 0
    iou_state = False
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
    if iou >= threshold_iou and area_ratio < 2:
        iou_state = True
    return iou_state


def compareTimeGap(time_1, time_2, threshold_t=0.1):
    t_state = False
    if abs(time_1 - time_2) > threshold_t:
        t_state = True
    return t_state


def compareRadian(radian1: float, radian2: float, thres_radian=0.1):
    """
    比较弧度值（用于能量机关）
    :param radian1: 弧度1
    :param radian2: 弧度2
    :param thres_radian: 阈值上限
    """
    t_state = False
    if calRadianGap(radian1, radian2) < thres_radian:
        t_state = True
    # else:
    #     print(radian1,radian2)
    return t_state


def compareHeight(height_min_thres: int, height_max_thres: int, armour1, armour2: list):
    """
    比较高度差
    @param height_min_thres: 高度差阈值下限
    @param height_max_thres: 高度差阈值上限
    @param armour2: ((x, y), (w, h), 0)
    @param armour1: ((x, y), (w, h), 0)
    """
    height_state = False
    y1, y2 = armour1[0][1], armour2[0][1]
    if height_min_thres <= abs(y1 - y2) <= height_max_thres:
        height_state = True
    return height_state


def compareWidth(width_min_thres, width_max_thres, armour1: list, armour2: list):
    """
    比较宽度差
    :param width_min_thres: 宽度差阈值下限
    :param width_max_thres: 宽度差阈值上限
    @param armour2: ((x, y), (w, h), 0)
    @param armour1: ((x, y), (w, h), 0)
    """
    width_state = False
    x1, x2 = armour1[0][0], armour2[0][0]
    if width_min_thres <= abs(x1 - x2) <= width_max_thres:
        width_state = True
    return width_state


#  比较目标距离，距离近的优先级高
def compareDistance(distance_array: np.ndarray) -> np.ndarray:
    sort_distance_array = np.sort(distance_array)[::-1]
    compared_distance = np.array([np.array(np.where(sort_distance_array == i)).min(
    ) for i in distance_array]).flatten()  # 加一保证排序从1开始
    norm_distance = np.around(compared_distance / np.sum(compared_distance), 1)
    return compared_distance


def compareAngle(rect_rotate_info: np.ndarray) -> np.ndarray:
    # TODO check whether yaw,pitch,raw order is correct
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
