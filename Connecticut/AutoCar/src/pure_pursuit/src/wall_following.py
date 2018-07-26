#!/usr/bin/env python

import rospy
import numpy as np

from ac_msgs.msg import drive_params
from sensor_msgs.msg import LaserScan

from pure_pursuit import pure_pursuit
from calc_desired_torque import *
from calculate_goalpoint import *


def stop_car():
    pub = rospy.Publisher('drive_parameters', drive_params, queue_size=5)
    for i in range(50):
        msg = drive_params()
        msg.angle = 0
        msg.velocity = 0
        pub.publish(msg)


if __name__ == '__main__':
    pub_drive_param = rospy.Publisher('drive_parameters', drive_params)
    pub_goalpoint = rospy.Publisher('goal_point', PointStamped, queue_size=1)
    rospy.init_node('geometric_controller', anonymous=True)
    rospy.on_shutdown(stop_car)
    torque_calculator = TorqueCalculator()
    while not rospy.core.is_shutdown_requested():
        laser_data = rospy.client.wait_for_message('scan', LaserScan)

        goal_point = lidar_to_rear_axle_coords(calculate_goalpoint_distance_from_wall(laser_data, .1))
        goal_point_in_lidar = rear_axle_to_lidar_coords(goal_point)
        x = goal_point[0]
        y = goal_point[1]
        goalpoint_angle_rad = math.atan(y / x)
        distance_to_goalpoint = math.sqrt(x ** 2 + y ** 2)

        # Calculate the steering control output
        #   The coordinate system of Lidar and pure_pursuit is counterclockwise
        #   but the steering direction is clockwise, so we need to negate the output of pure_pursuit
        steering_percentage = -pure_pursuit(goalpoint_angle_rad, distance_to_goalpoint)

        # Calculate the torque control output
        torque_percentage = torque_calculator.calc_torque_time_based(15)

        p_x = goal_point_in_lidar[0]
        p_y = goal_point_in_lidar[1]
        header = Header(stamp=rospy.Time.now(), frame_id="laser")
        point = Point(p_x, p_y, 0)
        point_stamped = PointStamped(header=header, point=point)
        pub_goalpoint.publish(point_stamped)

        msg = drive_params()
        msg.angle = int(steering_percentage)
        msg.velocity = int(torque_percentage)
        pub_drive_param.publish(msg)

    pub_drive_param.publish(0, 0)
    pub_drive_param.unregister()
