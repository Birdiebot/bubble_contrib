'''
Author: HarryWen
Date: 2022-05-28 23:47:51
FilePath: /bubble/src/bubble_contrib/bubble_decision/bubble_decision/decision.py
LastEditors: HarryWen
LastEditTime: 2022-08-10 23:46:57
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import rclpy
from rclpy.node import Node
from bubble_decision.gameAction import *
from bubble_decision.gimbalAction import *


class Decision():
    def __init__(self, node, robot_type) -> None:
        self.node = node
        self.robot_type = robot_type
        self.initRobot(robot_type)

    def initRobot(self, name):
        if name == "sentry_up":
            self.gimbal = SentryGimbal(self.node, auto_shoot=True)
            self.game = SentryGameAction(self.node)

        elif name == "sentry_down":
            self.gimbal = SentryGimbal(self.node, auto_shoot=True)
            self.game = SentryGameAction(self.node)

        elif name == "infantry":
            self.gimbal = InfantryGimbal(self.node, auto_shoot=False)
            self.game = InfantryGameAction(self.node)
            
        elif name == "hero":
            self.gimbal = HeroGimbal(self.node, auto_shoot=True)
            self.game = HeroGameAction(self.node)

        elif name == "engineer":
            pass
        elif name == "air":
            pass
        elif name == "radar":
            pass
        elif name == "gather":
            pass
        elif name == "standard":
            pass


class RobotAPI(Node):
    def __init__(self):
        super().__init__("Decision")
        self.declare_parameter('robot_type', 'None')
        name = self.get_parameter(
            'robot_type').get_parameter_value().string_value
        self.robot_decision = Decision(self, name)


def main(args=None):
    rclpy.init(args=args)
    decision = RobotAPI()
    rclpy.spin(decision)
    decision.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
