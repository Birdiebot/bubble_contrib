'''
Author: HarryWen
Date: 2022-04-23 16:27:36
FilePath: /bubble/src/bubble_contrib/bubble_aiming/bubble_aiming/utils/DataType.py
LastEditors: HarryWen
LastEditTime: 2022-07-24 02:05:28
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import time
from enum import IntEnum
import copy
import cv2
from bubble_aiming.utils.Calculator import *
from bubble_aiming.utils.Converter import *

TARGET_INFO_POS = 0
GIMBAL_INFO_POS = 1

GIMBAL_POSE_YAW = 0
GIMBAL_POSE_PITCH = 1
GIMBAL_POSE_TIMESTAMP = 2


class Wise(IntEnum):
    clockwise = 0
    anticlockwise = 1


class RotationInfo(IntEnum):
    center = 0
    size = 1
    angle = 2


class Armour(IntEnum):
    time = 0
    type = 1
    rect_info = 2
    gimbal_info = 3


class Strip(IntEnum):
    time = 0
    type = 1
    rect_info = 2
    gimbal_info = 3


class Rune_List(IntEnum):
    time = 0
    target_type = 1
    rect_info = 2
    center_rect_info = 3
    velocity = 4
    line_radian = 5
    gimbal_info = 6

    
class FIFO_Queue(object):
    def __init__(self, max_size=0, data_list = []):
        self.data_list = copy.deepcopy(data_list)
        self.max_size = max_size

    def __call__(self):
        return self.list

    def __str__(self) -> str:
        return str(self.data_list)

    def __getitem__(self, index):
        if isinstance(index, int):
            return self.data_list[index]
        elif type(index) is slice:
            return self.data_list[index.start: index.stop: index.step]

    def append(self, data):
        if self.size() >= self.max_size:
            self.data_list.pop(0)
        self.data_list.append(data)

    def size(self):
        return len(self.data_list)

    def clear(self):
        self.data_list.clear()


class GimbalInfo():
    def __init__(self) -> None:
        self.stamp = 0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.ros_stamp = None

    def get_yaw(self):
        return self.yaw

    def get_pitch(self):
        return self.pitch

    def get_roll(self):
        return self.roll

    def get_stamp(self):
        return self.stamp

    def set_ros_stamp(self, time):
        self.ros_stamp = time

    def set_data(self, stamp, yaw, pitch, roll):
        self.stamp = stamp
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

    def get_data(self):
        return [self.stamp, self.yaw, self.pitch, self.roll]


class TargetInfo(object):
    def __init__(self, name) -> None:
        self.name = name

class Position(): 
    def __init__(self,yaw_angle,pitch_angle) -> None:
        self.yaw_angle = yaw_angle
        self.pitch_angle = pitch_angle

class ArmourInfo(TargetInfo):
    def __init__(self) -> None:
        super().__init__("Armour")
        self.stamp = None
        self.box_points = None
        self.bbox_rect_rotation = None
        self.target_type = None
        self.rvec = None
        self.yaw_angle = None
        self.pitch_angle = None
        self.roll_angle = 0
        self.distance = None
        self.pose = {"x": 0, "y": 0, "z": 0}

        # light strip in the armour
        self.strip = None

    def __str__(self) -> str:
        return str(["stamp:", self.stamp, "box_points:", self.box_points, "bbox_rect_rotation:", self.bbox_rect_rotation, "target_type:",
                    self.target_type, "rvec:", self.rvec, "yaw_angle:", self.yaw_angle, "pitch_angle:", self.pitch_angle, "roll_angle:", self.roll_angle, "distance:", self.distance, "pose", self.pose])

    def get_stamp(self):
        return self.stamp

    def get_box_points(self):
        return self.box_points

    def get_target_type(self):
        return self.target_type

    def get_strip(self):
        return self.strip

    def get_position_xyz(self):
        return [self.pose['x'], self.pose['y'], self.pose['z']]

    def set_strip(self, strip_list):
        self.strip = strip_list

    def get_rotation_rpy(self):
        return [self.pose['x'], self.pose['y'], self.pose['z']]

    def set_rotation_rpy(self, yaw, pitch, roll):
        self.yaw_angle = yaw
        self.pitch_angle = pitch
        self.roll_angle = roll

    def set_data(self, stamp, box_points, target_type):
        self.stamp = stamp
        self.box_points = [[int(pose.x), int(pose.y)] for pose in box_points]
        self.target_type = target_type
        self.bbox_rect_rotation = cv2.minAreaRect(
            np.array(self.box_points, dtype="float32"))

    def get_rect_rotation(self):
        return self.bbox_rect_rotation

    def set_position(self, rvec, tvec):
        self.rvec = rvec
        self.set_rotation_rpy(
            np.arctan(tvec[0] / tvec[2]),
            np.arctan(tvec[1] / tvec[2]),
            0
        )
        self.distance = np.linalg.norm(tvec)

        # convert camera coodinate to gimbal coodinate
        self.pose['x'] = tvec[0]
        self.pose['y'] = tvec[1]
        self.pose['z'] = tvec[2]


class StripInfo():
    def __init__(self):
        self.stamp = 0
        self.box_points = 0
        self.class_id = 0

    def get_stamp(self):
        return self.stamp

    def get_box_points(self):
        return self.box_points

    def get_class_id(self):
        return self.class_id

    def set_data(self, stamp, box_points, class_id):
        self.stamp = stamp
        self.box_points = [[int(pose.x), int(pose.y)] for pose in box_points]
        self.class_id = class_id

class RuneInfo(TargetInfo):
    def __init__(self) -> None:
        super().__init__("Rune")
        self.stamp = None
        self.tar_bbox_points = None
        self.tar_bbox_rotation = None
        self.center_bbox_points = None
        self.center_bbox_rotation = None
        self.line_radian = None
        self.target_type = None
        self.rvec = None
        self.yaw_angle = 0
        self.pitch_angle = 0
        self.roll_angle = 0
        self.distance = None
        self.pose = {"x": 0, "y": 0, "z": 0}

    def __str__(self) -> str:
        return str(
        ["stamp:", self.stamp, 
        "tar_bbox_points:", self.tar_bbox_points,
        "tar_bbox_rotation:", self.tar_bbox_rotation, 
        "center_bbox_points:", self.center_bbox_points, 
        "center_bbox_rotation:", self.center_bbox_rotation, 
        "target_type:", self.target_type, 
        "rvec:", self.rvec, 
        "yaw_angle:", self.yaw_angle, 
        "pitch_angle:", self.pitch_angle, 
        "roll_angle:", self.roll_angle, 
        "distance:", self.distance, 
        "pose", self.pose
        ])

    def get_stamp(self):
        return self.stamp

    def get_box_points(self):
        return self.tar_bbox_points
        
    def get_tar_bbox_points(self):
        return self.tar_bbox_points

    def get_tar_bbox_rotation(self):
        return self.tar_bbox_rotation

    def get_center_bbox_points(self):
        return self.center_bbox_points
    
    def get_center_bbox_rotation(self):
        return self.center_bbox_rotation

    def get_target_type(self):
        return self.target_type

    def get_line_radian(self):
        return self.line_radian
    
    def get_velocity(self):
        return self.velocity

    def set_line_radian(self, line_radian):
        self.line_radian = line_radian

    def set_tar_bbox_points(self, tar_bbox_points):
        self.tar_bbox_points = tar_bbox_points
        self.tar_bbox_rotation = cv2.minAreaRect(
            np.array(self.tar_bbox_points, dtype="float32"))

    def set_center_bbox_points(self, center_bbox_points):
        self.center_bbox_points = center_bbox_points
        self.center_bbox_rotation = cv2.minAreaRect(
            np.array(self.center_bbox_points, dtype="float32"))
    
    def set_target_type(self, target_type):
        self.target_type = target_type

    def set_velocity(self,velocity):
        self.velocity = velocity
        
    def set_data(self, stamp, tar_bbox_points, center_bbox_points, velocity, target_type):
        self.stamp = stamp
        self.tar_bbox_points = [[int(pose.x), int(pose.y)] for pose in tar_bbox_points]
        self.center_bbox_points = [[int(pose.x), int(pose.y)] for pose in center_bbox_points]
        self.target_type = target_type
        self.tar_bbox_rotation = cv2.minAreaRect(
            np.array(self.tar_bbox_points, dtype="float32"))
        self.center_bbox_rotation = cv2.minAreaRect(
            np.array(self.center_bbox_points, dtype="float32"))
        self.line_radian = calLineRadians(self.tar_bbox_rotation[0], self.center_bbox_rotation[0])
        self.velocity = velocity

    def set_position(self, rvec, tvec):
        self.rvec = rvec
        self.yaw_angle = np.arctan(tvec[0] / tvec[2])
        self.pitch_angle = np.arctan(tvec[1] / tvec[2])
        self.distance = np.linalg.norm(tvec)

        # convert camera coodinate to gimbal coodinate
        self.pose['x'] = tvec[0]
        self.pose['y'] = -tvec[1]
        self.pose['z'] = tvec[2]

class WaveFormInfo():
    def __init__(self) -> None:
        pass
    def set_data(self, stamp, type, vel):
        self.stamp = stamp
        self.type = type
        self.vel = vel

class FIFO_Queue(object):
    def __init__(self, max_size=0, data_list = []):
        self.data_list = copy.deepcopy(data_list)
        self.max_size = max_size

    def __call__(self):
        return self.list

    def __str__(self) -> str:
        return str(self.data_list)

    def __getitem__(self, index):
        if isinstance(index, int):
            return self.data_list[index]
        elif type(index) is slice:
            return self.data_list[index.start: index.stop: index.step]
    def pop(self,index):
        self.data_list.pop(index)
        
    def append(self, data):
        if self.size() >= self.max_size:
            self.data_list.pop(0)
        self.data_list.append(data)

    def size(self):
        return len(self.data_list)

    def clear(self):
        self.data_list.clear()

    def get_data(self):
        return self.data_list
