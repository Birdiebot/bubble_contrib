import time
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32
from tf2_ros import TransformException
from rclpy.time import Time

from bubble_aiming.compensator.compensator import *
from bubble_aiming.decision.armourDecision import *
from bubble_aiming.decision.runeDecision import *
from bubble_aiming.predictor.armourPredictor import *
from bubble_aiming.predictor.largeRunePredictor import *
from bubble_aiming.predictor.smallRunePredictor import *
from bubble_aiming.predictor.outpostPredictor import *
from bubble_aiming.utils.Calculator import *
from bubble_aiming.utils.Comparator import *
from bubble_aiming.utils.Converter import *
from bubble_aiming.utils.DataType import *
from bubble_aiming.utils.Filter import *

MAX_HISTORY_SIZE = 200



class ArmourProcess:
    def __init__(
        self,
        node: Node,
        gimbal_info: GimbalInfo,
        bullet_vel: float,
        image_weight: int = 640,
        image_hight: int = 480
    ) -> None:
        self.node = node
        self.gimbal_info = gimbal_info
        self.bullet_vel = bullet_vel
        self.image_size = (image_weight, image_hight)
        self.present = Position(self.gimbal_info.get_pitch(), self.gimbal_info.get_yaw())
        self.period = 0

        # armour/mode choice
        self.armourDecision = ArmourDecision()
        self.ballisticCompensator = BallisticCompensator()
        # predictor
        self.AP = ArmourPredictor(1199, 1225)
        self.GP = GyroPredictor()
        self.OP = OutpostPredictor((640, 480))


        # context info

        self.origin_armour_list = FIFO_Queue(MAX_HISTORY_SIZE)
        self.iou_armour_list = FIFO_Queue(MAX_HISTORY_SIZE)
        self.processed_armour_list = FIFO_Queue(MAX_HISTORY_SIZE)
        self.processed_gyro_list = FIFO_Queue(MAX_HISTORY_SIZE)


    def process(self, armour_list, strip_list) -> ArmourInfo:
        # choice a armour in armour list
        target = self.decideMulTar(armour_list)
        self.recordData(self.origin_armour_list,
                        target, self.gimbal_info, False)

        # select aiming mode
        mode = self.updateMode()
        # auto-aiming mode
        if mode == "aiming":
            target = self.aimingAmourMode(target)
        # anti-gyro mode
        elif mode == "gyro":
            target = self.aimingGyroMode(target)

        elif mode == "outpost":
            target = self.aimingOutpostMode(target)

        return target

    def updateMode(self) -> str:
        """update mode"""
        mode = "outpost"
        # TODO
        # add more mode select, defult aiming mode
        # self.alter_armour = False
        # self.mode = self.armourDecision.judgeMode(self.origin_armour_list)
        # self.alter_armour = self.armourDecision.judgeArmourAlteration()
        return mode

    def decideMulTar(self, armour_list: list) -> ArmourInfo:
        """choice a armour from armour list"""
        rect_center_list = [
            armour.get_rect_rotation()[0] for armour in armour_list]
        target_index = compareTargetCenter(rect_center_list, self.image_size)
        return armour_list[target_index]

    def aimingAmourMode(self, target):
        """自瞄装甲板模式"""
        # print("===START==")
        target.set_position(
            *cvtToRectPNP(target, self.node.gimbal_offset_info, self.node.camera_intrinsic)
        )

        # print("pnp: ", np.degrees(target.yaw_angle), np.degrees(target.pitch_angle))

        # self.doPredict(target)
        # compensation
        # target = adjustMotionPosition(
        #     self.get_trans_vec("gimbal", "base_link"),
        #     target,
        #     self.gimbal_info,
        #     self.node.game_param["chassis_info"]['chassis_target_linear_x'],
        #     self.node.bullet_vel
        # )
    
        target = self.ballisticCompensator.adjustBallistics(
            target, self.gimbal_info.get_pitch(), self.bullet_vel)

        # target = adjustBallistics_try(
        #     target, self.gimbal_info.get_pitch(), self.bullet_vel)

        return target

    def aimingGyroMode(self, armour_list, reset=False):
        pred_rect_point_info = [[0, 0], [0, 0], [0, 0], [0, 0]]
        if not reset:
            present_armour = armour_list[-1]
            gimbal_info = present_armour[Armour.gimbal_info]
            target_type = present_armour[Armour.type]
            rect_rotation_info = cvtToRectRotation(
                present_armour[Armour.rect_info])

            pred_rect_rotation_info = self.GP.predict(rect_rotation_info)
            pred_rect_point_info = cvtToRectPoint(pred_rect_rotation_info)
            self.recordData(
                self.processed_gyro_list,
                pred_rect_point_info, target_type,
                gimbal_info, self.time_stamp,
                check_data=False
            )  # TODO 查看装甲板type
            rect_pnp_info_list = interpolation(self.processed_gyro_list .getRectPnpInfo(to_list=True),
                                               self.debug_param["gyro_debug_info"]["magnification"])
        return rect_pnp_info_list, pred_rect_point_info

    def aimingStripMode(self, strip: list):
        rect_pnp_info_list = [0, 0, 0]
        pred_rect_point_info = [[0, 0], [0, 0], [0, 0], [0, 0]]

        if len(self.processed_armour_list) > 15:
            last_armour = self.processed_armour_list[-1]
            strip_pnp_info = cvtToRectPNP(
                strip[Strip.rect_info], 4)  # TODO 添加灯条pnp
            last_armour_pnp = cvtToRectPNP(
                last_armour[Strip.rect_info], last_armour[Strip.type])
            yaw_radian, pitch_radian = last_armour_pnp.yaw, last_armour_pnp.pitch
            strip_yaw_radian, strip_pitch_radian = strip_pnp_info.yaw, strip_pnp_info.pitch

            yaw_radian_gap = abs(strip_yaw_radian - yaw_radian)
            pitch_radian_gap = abs(strip_pitch_radian - pitch_radian)
            if yaw_radian_gap < 0.2 and pitch_radian_gap < 0.2:
                rect_pnp_info_list = [[0, yaw_radian, pitch_radian]]

        # return rect_pnp_info_list, pred_rect_point_info

    def doPredict(self, target):
        down_time = self.ballisticCompensator.calDownTime(
            target, self.gimbal_info.get_pitch(), self.bullet_vel)  # 计算弹丸下坠时间
            
        pred_rect_rotation_info = self.AP.predict(
            self.node, target, self.origin_armour_list[-1], down_time)

        # pred_rect_rotation_info = self.AP.predict(self.node, present_armour, last_armour,down_time + 3)  # 预测目标
        pred_rect_point_info = cvtToRectPoint(pred_rect_rotation_info)
        self.recordData(
            self.processed_armour_list,
            pred_rect_point_info,
            present_target_type,
            present_gimbal_info,
            present_time_stamp,
            check_data=False
        )

    def recordData(self, dst: FIFO_Queue, armour_info: ArmourInfo, gimbal_info: GimbalInfo, check_data=True) -> None:
        if check_data and dst.size() >= 1:
            # IoU check
            is_iou = compareIoU(
                dst[-1][TARGET_INFO_POS].bbox_rect_rotation, armour_info.bbox_rect_rotation, 0.05)
            # time gap check
            is_gap = compareTimeGap(
                dst[-1][GIMBAL_INFO_POS].stamp, armour_info.stamp, 0.08)

            # clear destination list, if target is not the same armour
            if not is_iou and is_gap:
                dst.clear()

        # add data to destination list
        dst.append([armour_info, gimbal_info])

    def get_trans_vec(self, to_frame_rel, from_frame_rel):
        if self.gimbal_info.ros_stamp is None:
            return
        try:
            trans = self.node.tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, Time())
        except TransformException as ex:
            self.node.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        return trans

    def aimingOutpostMode(self, target):
        self.present = Position(self.gimbal_info.get_pitch(), self.gimbal_info.get_yaw())
        target.set_position(
            *cvtToRectPNP(target, self.node.gimbal_offset_info, self.node.camera_intrinsic))
        # 得到后续处理数据
        period = self.OP.updatePeriod(target)
        period = 0.4
        if period != 0:
            self.OP.predict(target, period, self.bullet_vel)
        target = self.OP.targetSift(target,self.present)
        return target

class RuneProcess:
    def __init__(self, node, gimbal_info, bullet_vel, image_weight, image_hight) -> None:
        self.node = node
        self.gimbal_info = gimbal_info
        self.bullet_vel = bullet_vel
        self.image_size = (image_weight, image_hight)

        self.remain_time = 200  # 当前比赛剩余时间
        self.mode = "Small_Power_Rune"

        self.runeDecision = RuneDecision()

        self.smallRunePredictor = SmallRunePredictor()
        self.largeRunePredictor = LargeRunePredictor()

        self.origin_rune_list = []  # 记录完整数据
        self.proc_small_rune_list = []  # 记录处理数据
        self.proc_large_rune_list = []

    def process(self, rune_info_list):
        rune_info = rune_info_list[-1]
        pred_rect_point_info = [[0, 0], [0, 0], [0, 0], [0, 0]]

        self.recordData(
            self.origin_rune_list,
            rune_info,
            self.gimbal_info,
            velocity=0,
            check_data=False
        )
        if len(self.origin_rune_list) >= 10:
            self.updateMode()
            if self.mode == "Small_Power_Rune":
                rune_info, pred_rect_point_info = self.smallPowerRuneMode(
                    self.origin_rune_list, self.node.rune_debug_info["ahead_angle"])
            elif self.mode == "Large_Power_Rune":
                rune_info, pred_rect_point_info = self.largePowerRuneMode(
                    self.origin_rune_list, self.node.rune_debug_info["ahead_time"])
                # *********************Debug************************
                if len(self.proc_large_rune_list) > 30:
                    vel_list = [i[0].get_velocity()
                                for i in self.proc_large_rune_list]

                    vel = lowPassFilter(2, 0.03, vel_list)[-5]
                    msg = Float32()
                    msg.data = vel
                    # print(msg.data)
                    # self.node.arg_pub.publish(msg)
            if rune_info.get_box_points() != [[0, 0], [0, 0], [0, 0], [0, 0]]:
                rune_info.set_position(
                    *cvtToRectPNP(rune_info, self.node.gimbal_offset_info, self.node.camera_intrinsic)
                )
        return rune_info, pred_rect_point_info

    def smallPowerRuneMode(self, rune_list, predict_degree):
        pred_rect_point_info = [[0, 0], [0, 0], [0, 0], [0, 0]]
        predict_target = rune_list[-1][0]
        pred_rect_rotation_info = self.smallRunePredictor.predict(
            rune_list, self.wise, predict_degree)

        pred_rect_point_info = cvtToRectPoint(pred_rect_rotation_info)
        predict_target.set_tar_bbox_points(pred_rect_point_info)

        return predict_target, pred_rect_point_info

    def largePowerRuneMode(self, rune_list, predict_time):
        pred_rect_point_info = [[0, 0], [0, 0], [0, 0], [0, 0]]

        target, last_target = rune_list[-1][0], rune_list[-10][0]
        predict_target = target

        line_radian = target.get_line_radian()
        last_line_radian = last_target.get_line_radian()

        if self.checkData(last_line_radian, line_radian, last_target.get_stamp(), target.get_stamp()):
            time_gap = target.get_stamp() - last_target.get_stamp()
            velocity = calAngularVelocity(
                last_line_radian, line_radian, time_gap)
            if velocity != 0:
                self.recordData(
                    self.proc_large_rune_list,
                    target,
                    self.gimbal_info,
                    velocity=velocity,
                    check_data=False
                )

                if len(self.proc_large_rune_list) > 30:
                    pred_rect_rotation_info = self.largeRunePredictor.predict(
                        self.proc_large_rune_list, self.wise, predict_time)
                    pred_rect_point_info = cvtToRectPoint(
                        pred_rect_rotation_info)
                    predict_target.set_tar_bbox_points(pred_rect_point_info)
        # vel_list = rune_list.getRectVeloInfo() # 速度列表
        return predict_target, pred_rect_point_info

    def updateMode(self):
        self.mode = self.runeDecision.judgeMode(self.remain_time)
        self.wise = self.runeDecision.judgeDirectionRotation(
            [rune[0].get_line_radian() for rune in self.origin_rune_list])

    def checkData(self, last_radian, line_radian, last_time_stamp, time_stamp):
        state = True
        if not compareRadian(last_radian, line_radian, 0.8) or compareTimeGap(time_stamp, last_time_stamp, 0.6):
            state = False
        return state

    def recordData(self, dst: FIFO_Queue, rune_info: RuneInfo,  gimbal_info, velocity=0, check_data=False):
        """
        记录数据
        @return:
        """
        # if len(dst) == 0:
        #     dst.append([rune_info, gimbal_info])
        # else:
        rune_info.set_velocity(velocity)
        if len(dst) != 0 and rune_info.get_center_bbox_points() == [[0, 0], [0, 0], [0, 0], [0, 0]]:
            center_bbox_points = dst[-1][0].get_center_bbox_points()
            rune_info.set_center_bbox_points(center_bbox_points)
        dst.append([rune_info, gimbal_info])


class MineralProcess:
    def __init__(self) -> None:
        pass

    def recordData(self):
        pass

    def process(self):
        pass

    def updateData(self):
        pass
