from bubble_aiming.utils.Filter import *
from bubble_aiming.utils.DataType import *
from bubble_aiming.utils.Calculator import *


class ArmourPredictor():
    def __init__(self, horziontal_pixel_per_radian, vertical_pixel_per_radian) -> None:
        self.param_coefficient = [1, 1, 1, 0.01, 1]
        self.kf = KalmanFilter(4, self.param_coefficient)
        self.horziontal_pixel_per_radian = horziontal_pixel_per_radian  # 水平方向 单位弧度对应像素
        self.vertical_pixel_per_radian = vertical_pixel_per_radian  # 垂直方向 单位弧度对应像素

    def reset(self):
        self.kf.reset()

    def predict(self, node, present_armour, last_armour, ahead_time):
        present_armour_time, last_armour_time = present_armour[Armour.time], last_armour[Armour.time]
        present_armour_point, last_armour_point = present_armour[Armour.rect_info], last_armour[Armour.rect_info]
        present_target_type, last_target_type = present_armour[Armour.type], last_armour[Armour.type]
        present_gimbal_info, last_gimbal_info = present_armour[Armour.gimbal_info], last_armour[Armour.gimbal_info]

        gimbal_yaw, gimbal_pitch, gimbal_time_gap = present_gimbal_info
        last_gimbal_yaw, last_gimbal_pitch, last_gimbal_time_stamp = last_gimbal_info
        
        # 计算两帧装甲板时间差
        armour_time_gap = present_armour_time - last_armour_time
        gimbal_time_gap = gimbal_time_gap - last_gimbal_time_stamp  # 计算两帧云台时间差

        if armour_time_gap != 0 and gimbal_time_gap != 0:
            rect_center, last_rect_center = calBoxCenter(present_armour_point), calBoxCenter(last_armour_point)

            # 由于两帧 云台和装甲板的数据时间不完全统一，故需分别使用对应的时间差
            # gimbal_p_vel = (gimbal_yaw - last_gimbal_yaw) * \
            #     self.horziontal_pixel_per_radian * 1.5 / gimbal_time_gap  # 云台水平方向像素速度
            # gimbal_y_vel = -(gimbal_pitch - last_gimbal_pitch) * \
            #     self.vertical_pixel_per_radian * 2.0 / gimbal_time_gap  # 云台垂直方向像素速度

            gimbal_p_vel = -(gimbal_pitch - last_gimbal_pitch) / gimbal_time_gap  # 云台垂直方向像素速度
            gimbal_y_vel = (gimbal_yaw - last_gimbal_yaw) * gimbal_time_gap  # 云台水平方向像素速度


            rect_center_p, rect_center_y = cvtPixelToRadian(rect_center, (640, 480), (63.11,44.59),0)
            last_rect_center_p, last_rect_center_y = cvtPixelToRadian(last_rect_center, (640, 480), (63.11,44.59),0)
            
            armour_p_vel = (rect_center_p - last_rect_center_p)/armour_time_gap
            armour_y_vel = (rect_center_y - last_rect_center_y)/armour_time_gap


            final_x_vel, final_y_vel = armour_p_vel - \
                gimbal_p_vel, armour_y_vel - gimbal_y_vel

            # 消除云台自身运动
            if abs(armour_y_vel) < abs(gimbal_y_vel) or abs(final_x_vel) < 10:
                final_x_vel = 0
            if abs(armour_p_vel) < abs(gimbal_p_vel) or abs(armour_y_vel) < 10:
                final_y_vel = 0

            #*******************Debug**********************#
            # if final_x_vel!=0 and final_y_vel !=0:
            #     node.get_logger().info("*************************************")
            #     node.get_logger().info("final_x_vel %f,final_y_vel %f " % (final_x_vel, final_y_vel))
            #     node.get_logger().info("x_vel %f,y_vel %f " % (armour_p_vel, armour_y_vel))
            #     node.get_logger().info("gimbal_p_vel %f,gimbal_y_vel %f " % (gimbal_p_vel, gimbal_y_vel))
            #     node.get_logger().info("gimbal_time_gap %f,armour_time_gap %f " % (gimbal_time_gap, armour_time_gap))
            #     node.get_logger().info("*************************************")

            #**********************************************#

            predict_center = self.kf.process(
                [rect_center[0], rect_center[1], final_x_vel, final_y_vel], ahead_time)[0:2]  # TODO 修改提前量（时间）
            rect_rotation_info = cvtToRectRotation(present_armour_point)
            predict_rect = self.calPredictedTarRect(list(rect_rotation_info), predict_center)  # 预测击打点的矩形框

        else:
            predict_rect = cvtToRectRotation(present_armour_point)
        return list(predict_rect)

    def predictPnP(self, ahead_time):
        armour_list = self.getDataList()
        time_gap = armour_list.getRectTargetInfo(-1).time - armour_list.getRectTargetInfo(-5).time
        rect_pnp_info, last_rect_pnp_info = armour_list.getRectPnpInfo(-1), armour_list.getRectPnpInfo(-5)
        gimbal_info, last_gimbal_info = armour_list.getRectTargetInfo(-1).gimbal_info, armour_list.getRectTargetInfo(-5).gimbal_info
        gimbal_yaw, gimbal_pitch = gimbal_info
        last_gimbal_yaw, last_gimbal_pitch = last_gimbal_info

        rect_yaw, rect_pitch = rect_pnp_info.yaw, rect_pnp_info.pitch
        last_rect_yaw, last_rect_pitch = last_rect_pnp_info.yaw, last_rect_pnp_info.pitch
        gimbal_yaw_vel = (rect_yaw-last_rect_yaw)-(gimbal_yaw-last_gimbal_yaw)
        gimbal_pitch_vel = (rect_pitch-last_rect_pitch)-(gimbal_pitch-last_gimbal_pitch)
        predict_rect = self.kf.process([rect_yaw, rect_pitch, gimbal_yaw_vel, gimbal_pitch_vel],time_gap + ahead_time)[0:2]
        predict_rect.insert(0,rect_pnp_info.distance) # pnp类型
        return predict_rect


class GyroPredictor():
    def __init__(self):
        self.center_coord_list = []

    def predict(self,rect_rotation_info):
        mid_point = self.calMidPoint(rect_rotation_info)
        self.recCenterCoord(mid_point, 30)  # TODO 测试数值大小
        predict_center = calAverageMidPoint(self.center_coord_list)
        predict_rect = self.calPredictedTarRect(
            rect_rotation_info[0], predict_center)  # 预测击打点的矩形框
        return predict_rect

    @staticmethod
    def calMidPoint(gyro_armour_list):
        x1, y1 = gyro_armour_list[0][RotationInfo.center]
        x2, y2 = gyro_armour_list[-1][RotationInfo.center]
        center_coord = [int((x1 + x2) / 2), int((y1 + y2) / 2)]
        return center_coord

    def recCenterCoord(self, mid_point, list_size):
        self.center_coord_list.append(mid_point)
        if len(self.center_coord_list) >= list_size:
            self.center_coord_list.pop(0)
