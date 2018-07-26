import numpy as np
import time
from lidar_utils import *


class TorqueCalculator:
    """
    This class is used to maintain variables required in calculating the torque.
    """
    def __init__(self):
        """
        -
        """
        self._lastchangedtime = time.clock()
        self._lastreturnedtorque = 0

    def calc_torque_safe(self, laser_scan, max_velocity):
        """

        :param laser_scan:
        :param max_velocity:
        :return:
        """
        if get_lidar_point_at_angle(laser_scan, 0) > 4:
            self._lastreturnedtorque = max_velocity
            self._lastchangedtime = time.clock()
            return max_velocity
        else:
            return self.calc_torque_time_based(max_velocity)

    def calc_torque_time_based(self, max_velocity):
        """

        :param laser_data:
        :param steering_percentage:
        :param max_velocity:
        :return: torque_percentage
        """
        current_time = time.clock()
        delta_time_seconds = current_time - self._lastchangedtime
        if delta_time_seconds > 0.3:  # We need to change the message
            if self._lastreturnedtorque == 0:
                self._lastreturnedtorque = max_velocity
                self._lastchangedtime = current_time
                return max_velocity
            else:  # We sent the max_velocity
                self._lastreturnedtorque = 0
                self._lastchangedtime = current_time
                return 0
        else:  # Don't change yet
            return self._lastreturnedtorque


def calc_desired_torque1(laser_data, steering_percentage, max_velocity, delta_time_seconds, previous_torque_percentage):
    """
    :param laser_data:
    :param steering_percentage:
    :return: previous_torque_percentage, time_last_run
    """
    if previous_torque_percentage > 0:
        # Running Time
        if delta_time_seconds > 0.3:
            previous_torque_percentage = 0
            delta_time_seconds = 0
    else:
        # Coasting Time
        if delta_time_seconds > 0.3:
            previous_torque_percentage = 17
            delta_time_seconds = 0

    return previous_torque_percentage, delta_time_seconds


def calc_desired_torque2(laser_data, steering_percentage, max_velocity, prev_time, desired_torque_percentage):
    """
    :param laser_data:
    :param steering_percentage:
    :return:
    """

    front_distance = get_distance_at_angle(laser_data, 0)
    side_distance = get_distance_at_angle(laser_data, -90) + get_distance_at_angle(laser_data, 90)
    front_to_side_ratio = front_distance/side_distance
    print("ratio: ", front_to_side_ratio)

    current_time = time.clock()
    delta_time_seconds = current_time - prev_time
    if desired_torque_percentage > 0:
        # Running Time
        if delta_time_seconds > 0.4 and front_to_side_ratio < 1.5:
            desired_torque_percentage = 0
    else:
        # Coasting Time
        if delta_time_seconds > 0.3:
            desired_torque_percentage = 15

    return desired_torque_percentage, current_time


def small_velocity_control(laser_data, steering_percentage, max_velocity, prev_time, desired_torque_percentage):
    """
    :param laser_data:
    :param steering_percentage:
    :return:
    """

    velocity = 6
    max_slow_velocity = 7
    vel_ratio = velocity/max_slow_velocity/2 # do that a whole cycle is 0.5 seconds
    tuning_coefficient = 1
    duty_cycle = vel_ratio * tuning_coefficient

    current_time = time.clock()
    delta_time_seconds = current_time - prev_time
    if desired_torque_percentage > 0:
        # Running Time
        if delta_time_seconds > duty_cycle:
            desired_torque_percentage = 0
            prev_time = current_time
    else:
        # Coasting Time
        if delta_time_seconds > 0.5 - duty_cycle:
            desired_torque_percentage = 20 #big torque to avoid jittering
            prev_time = current_time

    return desired_torque_percentage


def calc_desired_torque4(laser_data, steering_percentage, max_velocity):
    """
    :param laser_data:
    :param steering_percentage:
    :return:
    """
    steering_percentage = abs(steering_percentage)
    # desired_torque = np.interp(steering_percentage, [0, 100], [15, 0])
    if steering_percentage > 75:
        desired_torque = 0
    else:
        desired_torque = np.interp(steering_percentage, [0, 100], [15, 7])
    return desired_torque
