"""
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
License: GNU General Public License v3.0.
See LICENSE file in root directory.

Author: your name your email
Date: 2022-08-03 12:51:21
FilePath: /bubble_bringup/home/nvidia/Desktop/Projects/PythonProjects/bubble/src/bubble_contrib/bubble_aiming/bubble_aiming/aimingNode.py
LastEditors: your name your email
LastEditTime: 2022-09-14 13:37:42
"""
'''
Author: Harry
Date: 2022-01-19 02:21:14
FilePath: /bubble_bringup/home/nvidia/Desktop/bubble/src/bubble_contrib/bubble_aiming/bubble_aiming/aimingNode.py
LastEditors: HarryWen
LastEditTime: 2022-08-11 01:57:18
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import os
from collections import OrderedDict, deque

import message_filters
import rclpy
from bboxes_ex_msgs.msg import BoundingPolygonBox2D, BoundingPolygonBoxes2D
from game_msgs.msg import GameStatus, RobotHP
from geometry_msgs.msg import Point32
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange,
                                ParameterDescriptor, SetParametersResult)
from rclpy.node import Node
from rmctrl_msgs.msg import Chassis, Gimbal, Shooter
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int8
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from bubble_aiming.aimingProcess import *


class DPNode(Node):
    def __init__(self):
        super().__init__("autoaiming")
        # init param
        self.init_protocol_param()
        self.init_debug_params()
        if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
            self.set_parameters_callback(self.parameters_callback)
        else:
            self.add_on_set_parameters_callback(self.parameters_callback)

        # robot state subscriber
        self.gimbal_sub = self.create_subscription(
            Gimbal, '/status/gimbal', self.gimbal_callback, 10)
        self.game_status_sub = self.create_subscription(
            GameStatus, '/status/game', self.game_status_callback, 10)
        self.hp_sub = self.create_subscription(
            RobotHP, '/status/robotHP', self.hp_callback, 10)
        self.bullet_sub = self.create_subscription(
            Shooter, '/status/barrel', self.bullet_vel_callback, 10)
        self.chassis_sub = self.create_subscription(
            Chassis, '/status/chassis', self.chassis_callback, 10)

        # armour subscriber
        if self.use_synchronize:
            # sync gimbal and armour masseges
            gimbal_sub = message_filters.Subscriber(
                self, Gimbal, "/status/gimbal")
            armour_sub = message_filters.Subscriber(
                self, BoundingPolygonBoxes2D, "/cv/armour")
            ts = message_filters.ApproximateTimeSynchronizer(
                [armour_sub, gimbal_sub], 10, 0.1)
            ts.registerCallback(self.synchronizeData)
        else:
            # use armour massege directly
            self.armour_sub = self.create_subscription(
                BoundingPolygonBoxes2D, '/cv/armour', self.armour_callback, 1)

        # rune subscriber
        self.rune_sub = self.create_subscription(
            BoundingPolygonBoxes2D, '/cv/rune', self.rune_callback, 1)

        # init publisher
        self.gimbal_pub = self.create_publisher(
            Gimbal, '/decision/gimbal_api', 10)
        self.pred_armour_pub = self.create_publisher(
            BoundingPolygonBoxes2D, '/debug/predict', 10)
        self.arg_pub = self.create_publisher(Float32, '/debug/arg', 10)

        fps = 30
        self.create_timer(1/fps, self.pub_timer) #  lock frame number to fps

        # robot state setting
        self.gimbal_info = GimbalInfo()
        self.bullet_vel = 28
        self.image_weight = 1280
        self.image_hight = 1024

        self.armourProcess = ArmourProcess(
            self, self.gimbal_info, self.bullet_vel, self.image_weight, self.image_hight)
        self.runeProcess = RuneProcess(
            self, self.gimbal_info, self.bullet_vel, self.image_weight, self.image_hight)

        # tf trasform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_target_list = []


    def synchronizeData(self, armour_msg, gimbal_msg):
        self.gimbal_callback(gimbal_msg)
        self.armour_callback(armour_msg)
        self.process()

    def armour_callback(self, armour_msg):
        armour_list, strip_list = self.parse_armour(armour_msg)
        if not armour_list:
            return
        armour = self.armourProcess.process(armour_list, strip_list)

        # publish gimbal control data
        if not self.use_regular_send:
            self.last_target_list.append(armour)
        else:
            self.pub_gimbal_data(armour)

        # for debug
        if self.debug_mode:
            # publish predict data
            self.pub_predict_data(armour)

    def rune_callback(self, rune_msg):
        rune_list = self.parse_rune(rune_msg)
        if not rune_list:
            return

        rune, pred_rect_point_info = self.runeProcess.process(rune_list)
        if self.use_regular_send:
            self.last_target_list.append(rune)
        else:
            self.pub_gimbal_data(rune)

        if self.debug_mode:
            self.pub_predict_data(pred_rect_point_info)

    def pub_timer(self):
        if self.last_target_list:
            target = self.last_target_list.pop()
            if time.time()-target.stamp < 0.5:
                self.pub_gimbal_data(target)
            else:
                self.last_target_list.clear()


    def pub_gimbal_data(self, armour):
        gimbal_msg = Gimbal()
        gimbal_msg.mode = 1
        gimbal_msg.header.stamp = self.get_clock().now().to_msg()
        gimbal_msg.yaw = float(1*(armour.yaw_angle))
        gimbal_msg.pitch = float(1*(armour.pitch_angle))
        gimbal_msg.roll = float(1*(armour.roll_angle))
        self.gimbal_pub.publish(gimbal_msg)

   
    def pub_predict_data(self, armour):
        box = BoundingPolygonBox2D()
        for j in range(4):
            point = armour[j]
            pointMsg = Point32()
            pointMsg.x = float(point[0])
            pointMsg.y = float(point[1])
            box.pose.points.append(pointMsg)

        msg = BoundingPolygonBoxes2D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.bounding_boxes = [box]
        self.pred_armour_pub.publish(msg)

    def parse_armour(self, data: BoundingPolygonBoxes2D):
        armour_list = []
        strip_list = []

        target_time_stamp = data.image_header.stamp.sec + \
            data.image_header.stamp.nanosec * 1e-9
        for target in data.bounding_boxes:
            if target.class_id == "Armour":
                armour = ArmourInfo()
                armour.set_data(target_time_stamp,
                                target.pose.points, target.type)
                armour_list.append(armour)

            elif target.class_id == "Strip":
                # TODO add light strip parse
                pass
        return armour_list, strip_list

    def parse_rune(self, data: BoundingPolygonBoxes2D):
        temp_rune = [None, None]  # tar and center
        rune_list = []
        target_time_stamp = data.image_header.stamp.sec + \
            data.image_header.stamp.nanosec * 1e-9
        rune = RuneInfo()

        for target in data.bounding_boxes:
            if target.class_id == "Rune":
                if target.type == "armour" and [[int(pose.x), int(pose.y)] for pose in target.pose.points] != [[0, 0], [0, 0], [0, 0], [0, 0]]:
                    temp_rune[0] = target.pose.points
                elif target.type == "center" and [[int(pose.x), int(pose.y)] for pose in target.pose.points] != [[0, 0], [0, 0], [0, 0], [0, 0]]:
                    temp_rune[1] = target.pose.points
        if temp_rune[0] is not None and temp_rune[1] is not None:
            rune.set_data(target_time_stamp, temp_rune[0], temp_rune[1], 0, 3)
            rune_list.append(rune)

        return rune_list

    def bullet_vel_callback(self, data: Shooter):
        self.bullet_vel = data.bullet_vel
        if self.bullet_vel == 0:
            self.bullet_vel = 28

    def gimbal_callback(self, data: Gimbal):
        self.gimbal_info.set_data(
            data.header.stamp.sec + data.header.stamp.nanosec * 1e-9,
            data.yaw, data.pitch, data.roll
        )
        self.gimbal_info.set_ros_stamp(data.header.stamp)

    def chassis_callback(self, data: Chassis):
        self.game_param["chassis_info"]['chassis_target_linear_x'] = data.chassis_target_linear_x
        self.game_param["chassis_info"]['chassis_target_linear_y'] = data.chassis_target_linear_y
        self.game_param["chassis_info"]['chassis_target_linear_z'] = data.chassis_target_linear_z
        self.game_param["chassis_info"]['chassis_target_angular_x'] = data.chassis_target_angular_x
        self.game_param["chassis_info"]['chassis_target_angular_y'] = data.chassis_target_angular_y
        self.game_param["chassis_info"]['chassis_target_angular_z'] = data.chassis_target_angular_z

    def game_status_callback(self, data):
        self.game_param["game_status_info"]["game_type"] = data.game_type
        self.game_param["game_status_info"]["game_progress"] = data.game_progress
        self.game_param["game_status_info"]["stage_remain_time"] = data.stage_remain_time

    def hp_callback(self, data):
        self.game_param["robot_HP_info"]["red_1_robot_HP"] = data.red_1_robot_hp
        self.game_param["robot_HP_info"]["red_2_robot_HP"] = data.red_2_robot_hp
        self.game_param["robot_HP_info"]["red_3_robot_HP"] = data.red_3_robot_hp
        self.game_param["robot_HP_info"]["red_4_robot_HP"] = data.red_4_robot_hp
        self.game_param["robot_HP_info"]["red_5_robot_HP"] = data.red_5_robot_hp
        self.game_param["robot_HP_info"]["red_7_robot_HP"] = data.red_7_robot_hp
        self.game_param["robot_HP_info"]["red_outpost_HP"] = data.red_outpost_hp
        self.game_param["robot_HP_info"]["red_base_HP"] = data.red_base_hp
        self.game_param["robot_HP_info"]["blue_1_robot_HP"] = data.blue_1_robot_hp
        self.game_param["robot_HP_info"]["blue_2_robot_HP"] = data.blue_2_robot_hp
        self.game_param["robot_HP_info"]["blue_3_robot_HP"] = data.blue_3_robot_hp
        self.game_param["robot_HP_info"]["blue_4_robot_HP"] = data.blue_4_robot_hp
        self.game_param["robot_HP_info"]["blue_5_robot_HP"] = data.blue_5_robot_hp
        self.game_param["robot_HP_info"]["blue_7_robot_HP"] = data.blue_7_robot_hp
        self.game_param["robot_HP_info"]["blue_outpost_HP"] = data.blue_outpost_hp
        self.game_param["robot_HP_info"]["blue_base_HP"] = data.blue_base_hp

    def init_protocol_param(self):
        """
        Initialize the communication params
        """
        robot_HP_info = OrderedDict()
        robot_HP_info["red_1_robot_HP"] = 0
        robot_HP_info["red_2_robot_HP"] = 0
        robot_HP_info["red_3_robot_HP"] = 0
        robot_HP_info["red_4_robot_HP"] = 0
        robot_HP_info["red_5_robot_HP"] = 0
        robot_HP_info["red_7_robot_HP"] = 0
        robot_HP_info["red_outpost_HP"] = 0
        robot_HP_info["red_base_HP"] = 0
        robot_HP_info["blue_1_robot_HP"] = 0
        robot_HP_info["blue_2_robot_HP"] = 0
        robot_HP_info["blue_3_robot_HP"] = 0
        robot_HP_info["blue_4_robot_HP"] = 0
        robot_HP_info["blue_5_robot_HP"] = 0
        robot_HP_info["blue_7_robot_HP"] = 0
        robot_HP_info["blue_outpost_HP"] = 0
        robot_HP_info["blue_base_HP"] = 0

        game_status_info = OrderedDict()
        game_status_info["game_type"] = []
        game_status_info["game_progress"] = []
        game_status_info["stage_remain_time"] = []

        chassis_info = OrderedDict()
        chassis_info['chassis_target_linear_x'] = 0
        chassis_info['chassis_target_linear_y'] = 0
        chassis_info['chassis_target_linear_z'] = 0
        chassis_info['chassis_target_angular_x'] = 0
        chassis_info['chassis_target_angular_y'] = 0
        chassis_info['chassis_target_angular_z'] = 0

        self.game_param = OrderedDict()
        self.game_param["game_status_info"] = game_status_info
        self.game_param["robot_HP_info"] = robot_HP_info
        self.game_param["chassis_info"] = chassis_info

    def init_debug_params(self):
        """
        Initialize the debug params
        """

        self.declare_parameter('use_synchronize', False)
        self.use_synchronize = self.get_parameter('use_synchronize').value
        self.declare_parameter('use_regular_send', False)
        self.use_regular_send = self.get_parameter('use_regular_send').value
        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').value

        # Rune parameter descriptor
        angleFloatDescriptor = ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=45.0, step=0.1)])
        timeFloatDescriptor = ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)])
        
        # Rune debug param
        self.rune_debug_info = OrderedDict()
        self.rune_debug_info["ahead_time"] = 0.0
        self.rune_debug_info["ahead_angle"] = 0.0
        self.declare_parameter('ahead_time', 0.0, timeFloatDescriptor)
        self.declare_parameter('ahead_angle', 0.0, angleFloatDescriptor)

        # Anti-gyro parameter descriptor
        gyroFloatDescriptor = ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.1)])
        gyroLowDescriptor = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=100, step=1)])
        gyroHighDescriptor = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=100, to_value=1000, step=1)])

        # Anti-gyro debug param
        self.gyro_debug_info = OrderedDict()
        self.gyro_debug_info["magnification"] = 0
        self.gyro_debug_info["moving_alter_thres"] = 0
        self.gyro_debug_info["gyro_thres"] = 0
        self.gyro_debug_info["tolerance_thres"] = 0
        self.gyro_debug_info["armour_alter_thres"] = 0
        self.gyro_debug_info["height_differ_min_thres"] = 0
        self.gyro_debug_info["height_differ_max_thres"] = 0
        self.gyro_debug_info["width_differ_min_thres"] = 0
        self.gyro_debug_info["width_alter_min_thres"] = 0
        self.gyro_debug_info["width_differ_max_thres"] = 0
        self.declare_parameter('moving_alter_thres', 0, gyroLowDescriptor)
        self.declare_parameter('gyro_thres', 0, gyroLowDescriptor)
        self.declare_parameter('tolerance_thres', 0, gyroLowDescriptor)
        self.declare_parameter('armour_alter_thres', 0, gyroLowDescriptor)
        self.declare_parameter('height_differ_min_thres', 0, gyroLowDescriptor)
        self.declare_parameter('height_differ_max_thres', 0, gyroLowDescriptor)
        self.declare_parameter('width_differ_min_thres', 0, gyroLowDescriptor)
        self.declare_parameter('width_differ_max_thres', 10, gyroHighDescriptor)
        self.declare_parameter('width_alter_min_thres',10, gyroHighDescriptor)
        self.declare_parameter('gyro_iou_thres', 0.0, gyroFloatDescriptor)

        # Read params value from config
        self.declare_parameter('magnification', 0, descriptor=ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=10, step=1)]))
        self.gyro_debug_info["magnification"] = self.get_parameter(
            'magnification').value
        self.gyro_debug_info["moving_alter_thres"] = self.get_parameter(
            'moving_alter_thres').value
        self.gyro_debug_info["gyro_thres"] = self.get_parameter(
            'gyro_thres').value
        self.gyro_debug_info["tolerance_thres"] = self.get_parameter(
            'tolerance_thres').value
        self.gyro_debug_info["armour_alter_thres"] = self.get_parameter(
            'armour_alter_thres').value
        self.gyro_debug_info["height_differ_min_thres"] = self.get_parameter(
            'height_differ_min_thres').value
        self.gyro_debug_info["height_differ_max_thres"] = self.get_parameter(
            'height_differ_max_thres').value
        self.gyro_debug_info["width_differ_min_thres"] = self.get_parameter(
            'width_differ_min_thres').value
        self.gyro_debug_info["width_alter_min_thres"] = self.get_parameter(
            'width_alter_min_thres').value
        self.gyro_debug_info["width_differ_max_thres"] = self.get_parameter(
            'width_differ_max_thres').value
        self.gyro_debug_info["gyro_iou_thres"] = self.get_parameter(
            'gyro_iou_thres').value

        # camera offset and intrtnsic params
        self.gimbal_offset_info = OrderedDict()
        self.gimbal_offset_info["x_offset"] = 0
        self.gimbal_offset_info["y_offset"] = 0
        self.gimbal_offset_info["z_offset"] = 0

        camera_offset_descriptor = ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=-1.0, to_value=1.0, step=0.0001)])
        self.declare_parameter(
            'x_offset', 0.0, descriptor=camera_offset_descriptor)
        self.declare_parameter(
            'y_offset', 0.0, descriptor=camera_offset_descriptor)
        self.declare_parameter(
            'z_offset', 0.0, descriptor=camera_offset_descriptor)
        self.gimbal_offset_info["x_offset"] = self.get_parameter(
            'x_offset').value
        self.gimbal_offset_info["y_offset"] = self.get_parameter(
            'y_offset').value
        self.gimbal_offset_info["z_offset"] = self.get_parameter(
            'z_offset').value

        self.declare_parameter('camera_intrinsic_config_path', "")
        self.camera_intrinsic = self.get_parameter(
            'camera_intrinsic_config_path').value
        with open(self.camera_intrinsic, 'r') as f:
            self.camera_intrinsic = f.read()
        self.camera_intrinsic = eval(self.camera_intrinsic)

        self.rune_debug_info["ahead_time"] = self.get_parameter(
            'ahead_time').value
        self.rune_debug_info["ahead_angle"] = self.get_parameter(
            'ahead_angle').value

        gimbal_sense_descriptor = ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)])
        self.declare_parameter('yaw_sense', 0.0, descriptor=gimbal_sense_descriptor)
        self.declare_parameter('pitch_sense', 0.0, descriptor=gimbal_sense_descriptor)
        
        self.yaw_sense = self.get_parameter('yaw_sense').value
        self.pitch_sense = self.get_parameter('pitch_sense').value

    def parameters_callback(self, data):
        for param in data:
            if param.name == "magnification":
                self.gyro_debug_info["magnification"] = param.value
            elif param.name == "moving_alter_thres":
                self.gyro_debug_info["moving_alter_thres"] = param.value
            elif param.name == "gyro_thres":
                self.gyro_debug_info["gyro_thres"] = param.value
            elif param.name == "tolerance_thres":
                self.gyro_debug_info["tolerance_thres"] = param.value
            elif param.name == "armour_alter_thres":
                self.gyro_debug_info["armour_alter_thres"] = param.value
            elif param.name == "height_differ_min_thres":
                self.gyro_debug_info["height_differ_min_thres"] = param.value
            elif param.name == "height_differ_max_thres":
                self.gyro_debug_info["height_differ_max_thres"] = param.value
            elif param.name == "width_differ_min_thres":
                self.gyro_debug_info["width_differ_min_thres"] = param.value
            elif param.name == "width_alter_min_thres":
                self.gyro_debug_info["width_alter_min_thres"] = param.value
            elif param.name == "width_differ_max_thres":
                self.gyro_debug_info["width_differ_max_thres"] = param.value
            elif param.name == "gyro_iou_thres":
                self.gyro_debug_info["gyro_iou_thres"] = param.value
            elif param.name == "x_offset":
                self.gimbal_offset_info["x_offset"] = param.value
            elif param.name == "y_offset":
                self.gimbal_offset_info["y_offset"] = param.value
            elif param.name == "z_offset":
                self.gimbal_offset_info["z_offset"] = param.value
            elif param.name == "debug_mode":
                self.debug_mode = param.value
            elif param.name == "ahead_time":
                self.rune_debug_info["ahead_time"] = param.value
            elif param.name == "ahead_angle":
                self.rune_debug_info["ahead_angle"] = param.value

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    data_publisher = DPNode()
    rclpy.spin(data_publisher)
    data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
