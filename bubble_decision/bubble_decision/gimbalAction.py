'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-30 01:09:37
FilePath: /bubble/src/bubble_contrib/bubble_decision/bubble_decision/gimbalAction.py
LastEditors: HarryWen
LastEditTime: 2022-08-18 17:26:13
E-mail: robomaster@birdiebot.top
'''
import time

import std_msgs.msg
import rmctrl_msgs.msg

from rclpy.node import Node
from std_msgs.msg import Bool

GimbalInfo = {
    "last_time": 0,
    "times": 0,
}


class GimbalAction():
    def __init__(self, _node: Node, auto_shoot=False) -> None:
        self._node = _node
        self.auto_shoot = auto_shoot
        self.gimbal_sub = self._node.create_subscription(
            rmctrl_msgs.msg.Gimbal, '/decision/gimbal_api', self.gimbal_callback, 1)

        self.shooter_sub = self._node.create_subscription(
            Bool, '/decision/shooter_api', self.shooter_callback, 1)

        self.barrel_sub = self._node.create_subscription(
            rmctrl_msgs.msg.Shooter, '/status/barrel', self.barrel_callback, 1)

        self.gimbal_pub = self._node.create_publisher(
            rmctrl_msgs.msg.Gimbal, '/core/gimbal_api', 10)
        self.shooter_pub = self._node.create_publisher(
            rmctrl_msgs.msg.Shooter, '/core/shooter_api', 1)
        self.mode_pub = self._node.create_publisher(
            std_msgs.msg.Int8, '/core/mode_api', 10)
        self.gimbalInfo = GimbalInfo

    def shooter_callback(self, msg):
        if self.auto_shoot:
            print(msg.data)
            self.isShoot(msg.data)

    def barrel_callback(self, msg: rmctrl_msgs.msg.Gimbal) -> None:
        self.is_shoot = msg.is_shoot
        self.bullet_vel = msg.bullet_vel
        self.remain_bullet = msg.remain_bullet

    def gimbal_callback(self, msg):
        self.gimbal_pub.publish(msg)
        self.gimbalInfo['times'] += 1
        self.gimbalInfo["last_time"] = time.time()

    def isShoot(self, is_shoot: bool):
        shooter_msg = rmctrl_msgs.msg.Shooter()
        shooter_msg.is_shoot = is_shoot
        self.shooter_pub.publish(shooter_msg)

    def resetGimbal(self):
        gimbal_msg = rmctrl_msgs.msg.Gimbal()
        gimbal_msg.mode = 0
        self.gimbal_pub.publish(gimbal_msg)

    def modeCtrl(self, mode):
        msg = std_msgs.msg.Int8()
        msg.data = mode
        self.mode_pub.publish(msg)


class InfantryGimbal(GimbalAction):
    def __init__(self, _node: Node, auto_shoot=False) -> None:
        super().__init__(_node, auto_shoot)


class HeroGimbal(GimbalAction):
    def __init__(self, _node: Node, auto_shoot=True) -> None:
        super().__init__(_node, auto_shoot)



class SentryGimbal(GimbalAction):
    def __init__(self, _node: Node, auto_shoot=False) -> None:
        super().__init__(_node, auto_shoot)
        self.uartrx_timer = self._node.create_timer(0.1, self.doAction)

    def doAction(self):
        if 0 < time.time()-self.gimbalInfo['last_time'] < 0.3:
            if self.gimbalInfo['times'] > 3:
                self.modeCtrl(1)
                self.isShoot(True)
            elif self.gimbalInfo['times'] > 0:
                self.isShoot(False)
                self.resetGimbal()
                self.modeCtrl(1)
        elif 0.3 < time.time()-self.gimbalInfo['last_time'] < 1:
            self.isShoot(False)
        elif time.time()-self.gimbalInfo['last_time'] > 1:
            self.isShoot(False)
            self.resetGimbal()
            self.modeCtrl(0)
            self.gimbalInfo['times'] = 0

