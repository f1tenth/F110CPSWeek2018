#!/usr/bin/env python

import rospy
from ac_msgs.msg import drive_params
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point

import numpy as np

from calc_desired_torque import *
from lidar_utils import *


def lidar_to_rear_axle_coords(point):
    lidar_to_rear_axle_distance = 0.377

    # Lidar is in front of the rear axle (along the x-axis).
    # Hence the coordinates of a point in rear axle
    # has a lidar_to_rear_axle_distance meters offset in x values.
    return point + np.array([lidar_to_rear_axle_distance, 0])


def rear_axle_to_lidar_coords(point):
    lidar_to_rear_axle_distance = 0.377

    # Lidar is in front of the rear axle (along the x-axis).
    # Hence the coordinates of a point in rear axle
    # has a lidar_to_rear_axle_distance meters offset in x values.
    return point - np.array([lidar_to_rear_axle_distance, 0])


def get_path_length_at_angle(laser_data, alpha):
    p = lidar_to_rear_axle_coords(get_lidar_point_at_angle(laser_data, alpha))
    x = p[0]
    y = p[1]
    path_length = math.atan(y/x)*(x**2 + y**2)/y
    return path_length

def get_steering_percentage(laser_data):
    max_path_length = 0
    alpha_max = 0
    for alpha in range(-80, 80, 5):
        next_path_length = get_path_length_at_angle(laser_data, alpha)
        if max_path_length < next_path_length:
            max_path_length = next_path_length
            alpha_max = alpha
    p = lidar_to_rear_axle_coords(get_lidar_point_at_angle(laser_data, alpha_max))
    x = p[0]
    y = p[1]
    wheelbase = 0.33
    steering_angle_rad = math.atan(2*wheelbase*y/(x**2+y**2))
    steering_angle_deg = math.degrees(steering_angle_rad)

    # The wheels cannot physically turn more than 34 degrees
    max_turn_degrees = 34
    if steering_angle_deg < -max_turn_degrees:
        steering_angle_deg = -max_turn_degrees
    elif steering_angle_deg > max_turn_degrees:
        steering_angle_deg = max_turn_degrees

    # The steering command is in percentages
    steering_percentage = np.interp(steering_angle_deg, [-max_turn_degrees, max_turn_degrees], [-100, 100])
    return steering_percentage


def stop_car():
    pub = rospy.Publisher('drive_parameters', drive_params, queue_size=5)
    for i in range(50):
        msg = drive_params()
        msg.angle = 0
        msg.velocity = 0
        pub.publish(msg)


if __name__ == '__main__':
    prev_time = time.clock()
    pub_drive_param = rospy.Publisher('drive_parameters', drive_params, queue_size=1)
    pub_goalpoint = rospy.Publisher('goal_point', PointStamped, queue_size=1)
    rospy.init_node('geometric_controller', anonymous=True)
    rospy.on_shutdown(stop_car)

    while not rospy.core.is_shutdown_requested():
        laser_data = rospy.client.wait_for_message('scan', LaserScan)

        # Calculate the steering control output
        #   The coordinate system of Lidar and pure_pursuit is counterclockwise
        #   but the steering direction is clockwise, so we need to negate the output of pure_pursuit
        steering_percentage = -get_steering_percentage(laser_data)

        # Calculate the torque control output
        torque_percentage = calc_desired_torque1(laser_data, steering_percentage, 10)
#        torque_percentage = small_velocity_control(laser_data, steering_percentage, 15)

        goalpoint_in_lidar_coords = rear_axle_to_lidar_coords(goalpoint_in_rear_axel_coords)
        p_x = goalpoint_in_lidar_coords[0]
        p_y = goalpoint_in_lidar_coords[1]
        header = Header(stamp=rospy.Time.now(), frame_id="laser")
        point = Point(p_x, p_y, 0)
        point_stamped = PointStamped(header=header, point=point)
        pub_goalpoint.publish(point_stamped)

        msg = drive_params()
        msg.angle = steering_percentage
        msg.velocity = torque_percentage
        pub_drive_param.publish(msg)

    pub_drive_param.publish(0, 0)
    pub_drive_param.unregister()
