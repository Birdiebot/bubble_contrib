'''
Author: Harry
Date: 2022-01-20 14:55:58
FilePath: /bubble/src/bubble_contrib/bubble_aiming/bubble_aiming/utils/Filter.py
LastEditors: HarryWen
LastEditTime: 2022-05-29 14:34:23
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import numpy as np
from numpy.linalg import inv
from scipy import signal
from scipy.interpolate import make_interp_spline


class KalmanFilter():
    def __init__(self, param_num, param_coefficient):
        self._arg_dict = {"A": "", "B": "", "H": "", "Q": "", "R": ""}
        self.param_num = param_num
        self.param_coefficient = param_coefficient
        self.initParam(param_num, param_coefficient)
        self.is_init = False
        self.cur_estimate_x = None
        self.cur_best_estimate_x = None

        self.cur_estimate_p = None
        self.cur_best_estimate_p = None
    def reset(self):
        self.initParam(self.param_num,self.param_coefficient)
        
    def initParam(self, param_num: int, param_coefficient: float):
        """
        参数初始化
        :param param_num: 参数个数
        :param param_coefficient: 参数比例系数
        """

        for i, key in enumerate(self._arg_dict.keys()):
            self._arg_dict[key] = np.multiply(np.identity(param_num, dtype=int), np.array(param_coefficient[i]))
        self.A = self._arg_dict["A"]
        self.B = self._arg_dict["B"]
        self.H = self._arg_dict["H"]
        self.Q = self._arg_dict["Q"]
        self.R = self._arg_dict["R"]
        self.pre_best_estimate_x = np.zeros([1, param_num])
        self.pre_best_estimate_p = np.zeros([param_num, param_num])
        self.is_init = True

    def process(self, measure_state, down_time):
        if self.is_init:
            self.pre_best_estimate_x = np.array(measure_state)
        down_time_array = np.array([down_time for i in range(self.param_num // 2)])
        self.A = np.add(np.diag(down_time_array, k=self.param_num // 2), np.identity(self.param_num, dtype=float))

        self.prediction()
        self.update(measure_state)
        self.is_init = False
        return list(self.cur_best_estimate_x[0:self.param_num // 2])

    def update(self, measure_state):
        self.S = np.matmul(self.H, np.matmul(
            self.cur_estimate_p, self.H.T)) + self.R
        self.K = np.matmul(self.cur_estimate_p, np.matmul(self.H.T, inv(self.S)))  # kalman增益系数
        self.y = measure_state - np.dot(self.H, self.cur_estimate_x)

        self.cur_best_estimate_x = self.cur_estimate_x + np.dot(self.K, self.y)
        self.cur_best_estimate_p = np.matmul(np.eye(self.param_num) - np.matmul(self.K, self.H), self.cur_estimate_p)

        self.pre_best_estimate_x = self.cur_best_estimate_x
        self.pre_best_estimate_p = self.cur_best_estimate_p

    def prediction(self):
        self.cur_estimate_x = np.dot(self.A, self.pre_best_estimate_x.T) + np.dot(self.B, np.zeros(self.param_num))
        self.cur_estimate_p = np.matmul(self.A, np.matmul(self.pre_best_estimate_p, self.A.T)) + self.Q


def lowPassFilter(fps, signal_fps, data):
    """
    低通滤波器
    :param fps: 采样频率
    :param signal_fps:信号本身最大的频率
    :param data: 待过滤数据
    :return:
    """
    b, a = signal.butter(fps, signal_fps, 'lowpass')
    filtered_data = signal.filtfilt(b, a, data)
    return filtered_data


def mediumFilter(data, kernel_size=None) -> list:
    return signal.medfilt(data, kernel_size)  # 一维中值滤波


def movingAverageFilter(data_array: np.ndarray, w):
    """
    移动平均滤波器
    :param data_array: 数据数组
    :param w: 一次对w个数据进行移动平均
    :return:平滑数据后的数据
    """
    cal_gap = lambda x, len_x: sum([x[i + 1] - x[i] for i in range(len_x - 1)]) / (len_x - 1)
    gap = cal_gap(data_array, len(data_array))
    for i in range(w - 1):
        #  保证数据大于0
        if data_array[0] - gap >= 0:
            data_array = np.insert(data_array, 0, data_array[0] - gap)
        else:
            data_array = np.insert(data_array, 0, data_array[0])
    return np.convolve(data_array, np.ones(w), 'valid') / w


def interpolation(data_list: list, magnification: int):
    """
    数据插值器
    :param data_list: 数据列表(长度最小为4)
    :param magnification: n倍插值(将原有数据个数扩大至n倍)
    :return:扩大后的数据
    """
    interpolate_data_list = []
    data_num = len(data_list)  # 数据个数
    if data_num == 0:
        interpolate_data_list = [[0, 0, 0]]
    elif 1 <= data_num <= 5:
        interpolate_data_list = data_list[-2:] if data_num != 1 else data_list
    elif 5 < data_num:
        data_array = np.array([np.array(i) for i in data_list])
        data_shape = np.shape(data_array)  # 数组形状
        processed_data_list = [[] for i in range(data_shape[1])]
        #  提取数据进行处理
        for i in range(data_shape[1]):
            x_array, y_array = np.linspace(0, 1, data_num), data_array[:, i]
            filter_y_array = movingAverageFilter(y_array, 4)  # 数据平滑处理
            x_smooth = np.linspace(min(x_array), max(x_array), magnification * data_num)
            y_smooth = make_interp_spline(x_array, filter_y_array)(x_smooth)
            processed_data_list[i] = y_smooth
        interpolate_data_list = [np.array(processed_data_list)[:, j] for j in range(magnification * data_num)][-magnification:]
    return interpolate_data_list
