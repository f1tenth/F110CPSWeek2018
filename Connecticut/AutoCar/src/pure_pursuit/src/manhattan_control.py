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
    rospy.init_node('geometric_controller', anonymous=True)
    pub_drive_param = rospy.Publisher('drive_parameters', drive_params)
    pub_goalpoint = rospy.Publisher('goal_point', PointStamped, queue_size=1)
    rospy.on_shutdown(stop_car)
    torque_percentage = 0
    torque_calculator = TorqueCalculator()

    while not rospy.core.is_shutdown_requested():
        laser_data = rospy.client.wait_for_message('scan', LaserScan)
        prev_time = time.clock()  # calculate loop time

        goalpoint_at_infinity = calculate_goalpoint_double_manhattan(laser_data)

        scale = 1/math.sqrt(goalpoint_at_infinity[0]**2 + goalpoint_at_infinity[1]**2)
        goalpoint_in_rear_axel_coords = goalpoint_at_infinity*scale
        goalpoint_angle_rad = math.atan(goalpoint_at_infinity[1] / goalpoint_at_infinity[0])
        # Calculate the steering control output
        #   The coordinate system of Lidar and pure_pursuit is counterclockwise
        #   but the steering direction is clockwise, so we need to negate the output of pure_pursuit
        distance_to_goalpoint = 0.8
        steering_percentage = -pure_pursuit(goalpoint_angle_rad, distance_to_goalpoint)

        # Calculate velocity
        # velocity = calculate_velocity(goalpoint_at_infinity)
        # print("Goalpoint-based velocity (m/s): ", velocity)

        # Calculate the torque control output
        distance_to_goalpoint_at_infinity = goalpoint_at_infinity[0]**2+goalpoint_at_infinity[1]**2
        if distance_to_goalpoint_at_infinity > 8:
            torque_percentage = min(24, int(distance_to_goalpoint_at_infinity*0.8))
        else:
            torque_percentage = int(torque_calculator.calc_torque_time_based(20))
        # visualize the final goalpoint for purepursuit
        goalpoint_in_lidar_coords = rear_axle_to_lidar_coords(goalpoint_in_rear_axel_coords)
        p_x = goalpoint_in_lidar_coords[0]
        p_y = goalpoint_in_lidar_coords[1]
        header = Header(stamp=rospy.Time.now(), frame_id="laser")
        point = Point(p_x, p_y, 0)
        point_stamped = PointStamped(header=header, point=point)
        pub_goalpoint.publish(point_stamped)

        # send the torque and steering percentages to Teensy
        msg = drive_params()
        msg.angle = int(steering_percentage)
        msg.velocity = int(torque_percentage)
        pub_drive_param.publish(msg)

        # evaluate the execution time of the main control loop
        current_time = time.clock()
        delta_time_seconds = current_time - prev_time
        print("Main loop execution time in milliseconds: ", delta_time_seconds*1000)

    pub_drive_param.publish(0, 0)
    pub_drive_param.unregister()
