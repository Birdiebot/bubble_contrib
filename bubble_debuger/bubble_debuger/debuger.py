'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-17 02:23:17
FilePath: /bubble/src/bubble_contrib/bubble_debuger/bubble_debuger/debuger.py
LastEditors: HarryWen
LastEditTime: 2022-07-23 22:25:48
E-mail: robomaster@birdiebot.top
'''
import rclpy
from rclpy.node import Node

import message_filters

import bboxes_ex_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from bboxes_ex_msgs.msg import BoundingPolygonBoxes2D

import cv2


# class Debuger(Node):
#     def __init__(self):
#         super().__init__('Bubble_Debuger')
#         self.image_sub = message_filters.Subscriber(self, Image, "/image_topic")
#         info_sub = message_filters.Subscriber(
#             self, bboxes_ex_msgs.msg.BoundingPolygonBoxes2D, '/cv/armour')
#         image_sub = message_filters.Subscriber(self, Image, "/raw_image")
#         ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1)
#         ts.registerCallback(self.debuger_callback)
#         self.bridge = CvBridge()

#     def debuger_callback(self, image_msg, armour_msg):
#         image = self.bridge.imgmsg_to_cv2(image_msg)
#         # pts = np.array([[100,50],[200,300],[70,200],[50,100]],np.int32)
#         # pts = pts.reshape((-1,1,2))
#         # cv2.polylines(img,[pts],True,(0,255,255),2)
#         cv2.imshow("test", image)
#         cv2.waitKey(1)
#         print(armour_msg)


class DebugerAiming(Node):
    def __init__(self):
        super().__init__('Bubble_Debuger')
        # self.image_sub = self.create_publisher(Image, "/image_topic")
        self.image = None
        self.pts = None
        self.info_sub = self.create_subscription(BoundingPolygonBoxes2D, '/debug/predict',self.info_callback, 5)
        self.image_sub = self.create_subscription(Image, "/raw_image",self.image_callback,5)
        self.bridge = CvBridge()
        self.create_timer(0.01, self.show)  # 每0.2秒执行一次

    def info_callback(self, armour_msg):
        for target in armour_msg.bounding_boxes:
            box_points = [[int(pose.x), int(pose.y)] for pose in target.pose.points]
 
        self.pts = np.array(box_points)
        self.pts = self.pts.reshape((-1,1,2))

    def image_callback(self,image_msg):
        self.image = self.bridge.imgmsg_to_cv2(image_msg)

    
    def show(self):
        try:
            if self.image is not None and self.pts is not None:
                self.image = cv2.resize(self.image, (640,480))

                cv2.polylines(self.image,[self.pts],True,(0,255,255),2)
                cv2.imshow("test", self.image)
                cv2.waitKey(1)
                self.image = None
                self.pts =None
        except Exception:
            pass






# class DebugerAiming(Node):
#     def __init__(self):
#         super().__init__('Bubble_Debuger')
#         self.image = None
#         self.pts = None

#         # self.info_sub = self.create_subscription(BoundingPolygonBox2D, '/test',self.info_callback, 5)
#         # self.image_sub = self.create_subscription(Image, "/raw_image",self.image_callback,5)
        
#         info_sub = message_filters.Subscriber(self, BoundingPolygonBoxes2D, "/test")
#         image_sub = message_filters.Subscriber(self, Image, "/cv/armour")
#         ts = message_filters.ApproximateTimeSynchronizer([info_sub,image_sub], 10, 1000000)
#         ts.registerCallback(self.synchronizeData)
        
#         self.bridge = CvBridge()
#         # self.create_timer(0.01, self.show)  # 每0.2秒执行一次

#     def synchronizeData(self,rect_msg,image_msg):
#         self.info_callback(rect_msg)
#         self.image_callback(image_msg)
#         self.show()

#     def info_callback(self, armour_msg):
#         box_points = [[int(pose.x), int(pose.y)] for pose in armour_msg.pose.points]
#         self.pts = np.array(box_points)
#         self.pts = self.pts.reshape((-1,1,2))

#     def image_callback(self,image_msg):
#         self.image = self.bridge.imgmsg_to_cv2(image_msg)

    
#     def show(self):
#         try:
#             if self.image is not None and self.pts is not None:
#                 self.image = cv2.resize(self.image, (640,480))
#                 cv2.polylines(self.image,[self.pts],True,(0,255,255),2)
#                 cv2.imshow("test", self.image)
#                 cv2.waitKey(1)

#         except Exception:
#             pass

def main(args=None):
    rclpy.init(args=args)

    debuger = DebugerAiming()
    rclpy.spin(debuger)
    debuger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
