
'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-30 01:09:37
FilePath: /bubble/src/bubble_contrib/bubble_decision/bubble_decision/gameAction.py
LastEditors: HarryWen
LastEditTime: 2022-08-11 01:29:46
E-mail: robomaster@birdiebot.top
'''
import os
import re
import string

import rmctrl_msgs.msg
import message_filters

from rclpy.node import Node
from game_msgs.msg import GameStatus
from std_msgs.msg import Int8
from rcl_interfaces.msg import (
    IntegerRange, ParameterDescriptor, SetParametersResult)


class GameAction():
    def __init__(self, node: Node) -> None:
        self.node = node
        self.game_type = 0  # 比赛类型
        self.game_progress = 0 # 当前比赛阶段
        self.stage_remain_time = 220  # 当前阶段剩余时间
        self.manifold_ctrl = 1  # 模式控制
        # game_status_sub = message_filters.Subscriber(
        #     self, GameStatus, "/status/game")
        # manifold_ctrl_sub = message_filters.Subscriber(
        #     self, Int8, "/status/manifold_ctrl")
        # ts = message_filters.ApproximateTimeSynchronizer(
        #     [game_status_sub, manifold_ctrl_sub], 10, 0.1)
        # ts.registerCallback(self.game_status_callback)

        self.manifold_ctrl_sub = self.node.create_subscription(
            Int8, '/status/manifold_ctrl', self.manifold_ctrl_callback, 10)

        self.game_status_sub = self.node.create_subscription(
            GameStatus, '/status/game', self.game_status_callback, 10)

    def game_status_callback(self, game_status_msg):
        self.game_type = game_status_msg.game_type
        self.game_progress = game_status_msg.game_progress
        self.stage_remain_time = game_status_msg.stage_remain_time

    def manifold_ctrl_callback(self, manifold_ctrl_msg):
        self.manifold_ctrl = manifold_ctrl_msg.data

    def setParamValue(self, topic: string, param: string, value: float):
        if self.getParamValue(topic, param) != value:
            os.popen(
                f". '/home/nvidia/Desktop/bubble/install/setup.sh' && ros2 param set {topic} {param} {value}")

    def getParamValue(self, topic: string, param: string):
        res = os.popen(
            f". '/home/nvidia/Desktop/bubble/install/setup.sh' && ros2 param get {topic} {param}")
        result = ''.join(re.findall(r"\d+\.?\d*", res.read()))  # 最终设定的参数
        return result


class InfantryGameAction(GameAction):
    def __init__(self, _node: Node) -> None:
        self.node = _node
        self.init_camera_params()
        self.last_exposure_time = self.orin_exposure_time
        self.uartrx_timer = self.node.create_timer(0.1, self.doAction)
        super().__init__(_node)

    def doAction(self):
        if self.stage_remain_time < 200 or 240 < self.stage_remain_time < 360:
            if self.manifold_ctrl == 2 and self.last_exposure_time != self.rune_exposure_time:
                self.setParamValue("/camera", "exposureTime",
                                   self.rune_exposure_time)
                self.last_exposure_time = self.rune_exposure_time
        else:
            # self.status = "aimingArmour"
            if self.manifold_ctrl == 1 and self.last_exposure_time != self.armour_exposure_time:
                self.setParamValue("/camera", "exposureTime",
                                   self.armour_exposure_time)
                self.last_exposure_time = self.armour_exposure_time

    def init_camera_params(self):
        """
        Initialize the camera params
        """
        exposureTimeDescriptor = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=20000, step=1)])

        self.armour_exposure_time = 0
        self.rune_exposure_time = 0
        self.orin_exposure_time = 0

        self.node.declare_parameter('orin_exposure_time', 0, exposureTimeDescriptor)
        self.node.declare_parameter('armour_exposure_time',
                               0, exposureTimeDescriptor)
        self.node.declare_parameter('rune_exposure_time', 0, exposureTimeDescriptor)

        self.orin_exposure_time = self.node.get_parameter(
            'orin_exposure_time').value
        self.armour_exposure_time = self.node.get_parameter(
            'armour_exposure_time').value
        self.rune_exposure_time = self.node.get_parameter(
            'rune_exposure_time').value

    def parameters_callback(self, data):
        for param in data:
            if param.name == "rune_exposure_time":
                self.rune_exposure_time = param.value
            elif param.name == "armour_exposure_time":
                self.armour_exposure_time = param.value
            # elif param.name == "orin_exposure_time":
            #     self.orin_exposure_time = param.value

        return SetParametersResult(successful=True)


class SentryGameAction(GameAction):
    def __init__(self, _node: Node) -> None:
        super().__init__(_node)


class HeroGameAction(GameAction):
    def __init__(self, _node: Node) -> None:
        super().__init__(_node)
