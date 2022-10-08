from scipy import integrate
from bubble_aiming.utils.Filter import *
from bubble_aiming.utils.DataType import *
from bubble_aiming.utils.Calculator import *


class LargeRunePredictor():
    def __init__(self):
        self.wise = Wise.clockwise
        self.a, self.w = 0.785, 1.884

        self.filter_vel_list = FIFO_Queue(100)
        self.filter_time_list = FIFO_Queue(100)

        self.avg_w_list = FIFO_Queue(30)
        self.avg_a_list = FIFO_Queue(30)

        self.peak_time = None
        self.trough_time = None
        self.last_a = 0
        self.last_w = 0
        self.pred_list = []

    def predict(self, rune_list, wise, predict_time):
        predict_rect = [(0, 0), (0, 0), 0]
        rune_target = rune_list[-1]
        self.wise = wise
        aligned_time = 0
        filter_time_list, filter_vel_list = self.filterVel(
            rune_list, 0.03, -10)
        if filter_vel_list.size() > 40:

            # 找波峰和波谷
            peak_index = findPeak(filter_vel_list.get_data(), 1.9)
            trough_index = findTrough(filter_vel_list.get_data(), 0.55)
            # 若找到波峰
            if peak_index is not None:
                self.peak_vel = filter_vel_list[peak_index]
                self.peak_time = filter_time_list[peak_index]

            # 若找到波谷
            if trough_index is not None:
                self.trough_vel = filter_vel_list[trough_index]
                self.trough_time = filter_time_list[trough_index]

                self.a = np.round((2.090 - self.trough_vel) / 2, 2)
                if self.a != self.last_a:
                    self.avg_a_list.append(self.a)
                    self.last_a = self.a


            # 若存在波峰和波谷
            if (self.peak_time and self.trough_time) is not None:
                self.w = np.round(
                    math.pi / abs(self.trough_time - self.peak_time), 2)
                if 1.884 < self.w < 2 and self.w != self.last_w:
                    self.avg_w_list.append(self.w)
                    self.last_w = self.w
                elif self.peak_time is not None:
                    aligned_time = self.alignTime(
                        rune_target[0].get_stamp(), self.peak_time, 0, self.w)
                # 若只存在波谷
                elif self.trough_time is not None:
                    aligned_time = self.alignTime(
                        rune_target[0].get_stamp(), 0, self.trough_time, self.w)
                    

            # 若只存在波峰
            elif self.peak_time is not None:
                aligned_time = self.alignTime(
                    rune_target[0].get_stamp(), self.peak_time, 0, self.w)
            # 若只存在波谷
            elif self.trough_time is not None:
                aligned_time = self.alignTime(
                    rune_target[0].get_stamp(), 0, self.trough_time, self.w)


            if self.avg_w_list.size() > 0:
                self.a = np.array(self.avg_a_list.get_data()).mean()
                self.w = np.array(self.avg_w_list.get_data()).mean()
                aligned_time = self.alignTime(
                    rune_target[0].get_stamp(), self.peak_time, self.trough_time, self.w)
            
            def sine_func(t): return self.a * np.sin(self.w * t) + (2.09 - self.a)
            if aligned_time != 0:
                predict_radian, _ = integrate.quad(
                    sine_func, aligned_time, aligned_time + predict_time)  # integrate.quad()求一重积分
                predict_rect = self.calPredTarRect(rune_target, predict_radian)

            # ******Debug******
            # self.checkPredictData(
            #     rune_list[-1],
            #     predict_rect[RotationInfo.center],
            #     rune_target[0].get_center_bbox_rotation()[
            #         RotationInfo.center],
            #     predict_time)

        return predict_rect

    def checkPredictData(self, rune_target, predict_center, circle_center, predict_time):
        new_stamp = rune_target[0].get_stamp() + predict_time
        new_radian = calLineRadians(circle_center, predict_center)
        self.pred_list.append([new_stamp, new_radian])

        temp_index = np.argmin(
            np.abs(np.array([i[0] for i in self.pred_list])-rune_target[0].get_stamp()))
        if (abs(self.pred_list[temp_index][0]-rune_target[0].get_stamp())) < 0.02:
            print(rune_target[0].get_stamp(), calLineRadians(rune_target[0].get_center_bbox_rotation()[
                  0], rune_target[0].get_tar_bbox_rotation()[0]), self.pred_list[temp_index][1])
        if len(self.pred_list) > 1000:
            self.pred_list.pop(0)

    def generateSinFunc(self, a, w):
        def sine_func(t): return a * np.sin(w * t) + (2.09 - a)
        return sine_func

    def alignTime(self, pres_time, peak_time, trough_time, w):
        if peak_time > trough_time:
            t = math.pi / (2 * w)
            aligned_time = pres_time - (peak_time - t)
        else:
            t = 3 * math.pi / (2 * w)
            aligned_time = pres_time - (trough_time - t)
        return aligned_time

    def filterVel(self, rune_list, signal_fps, record_index):
        vel_list = [i[0].get_velocity() for i in rune_list]  # 速度列表
        filter_vel = lowPassFilter(2, signal_fps, vel_list)
        if self.filter_time_list.size() == 0:
            for i in range(0,len(rune_list)+record_index):
                self.filter_time_list.append(rune_list[i][0].get_stamp())
                self.filter_vel_list.append(filter_vel[i])
        else:
            self.filter_time_list.append(
                rune_list[record_index][0].get_stamp())
            self.filter_vel_list.append(filter_vel[record_index])

        return self.filter_time_list, self.filter_vel_list

    def calPredTarRect(self, rune_target: Rune_List, predict_radian):
        """
        计算预测矩形的旋转框信息
        @param rect_rotation_info: 原始矩形框信息
        @param pred_center: 预测矩形框中心
        @param pred_angle: 预测矩形框旋转角度
        @return: 预测矩形旋转框信息
        """
        if self.wise == Wise.anticlockwise:
            pred_angle = np.degrees(predict_radian)
        elif self.wise == Wise.clockwise:
            pred_angle = -np.degrees(predict_radian)

        rect_rotation_info = list(cvtToRectRotation(rigidTransform(
            pred_angle,
            rune_target[0].get_tar_bbox_rotation()[
                RotationInfo.center],
            rune_target[0].get_tar_bbox_points())))
        pred_center = self.calPredTarCenter(
            rune_target[0].get_center_bbox_rotation()[RotationInfo.center],
            rune_target[0].get_tar_bbox_rotation()[RotationInfo.center],
            pred_angle)

        pred_tar_rect = (
            pred_center, rect_rotation_info[RotationInfo.size], rect_rotation_info[RotationInfo.angle])

        return pred_tar_rect

    def calPredTarCenter(self, circle_center, tar_center, pred_angle):
        """
        计算预测矩形框的中心
        :param circle_center: 圆心
        :param pred_angle: 目标预测角度
        :return: 预测中心点坐标
        """
        radius = calPointDistance(circle_center, tar_center)  # 目标中心与旋转中心的距离
        predicted_tar_center = rigidTransform(
            pred_angle, circle_center, [tar_center])[0]
        return predicted_tar_center
