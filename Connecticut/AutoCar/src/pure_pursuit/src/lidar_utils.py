import numpy as np
import math

import rospy
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped, Point
from std_msgs.msg import Header

pub_rightwall = rospy.Publisher('rightwall/wall', PolygonStamped, queue_size=1)
pub_rightwall_endpoint = rospy.Publisher('rightwall/endpoint', PointStamped, queue_size=1)

pub_rightwall_2 = rospy.Publisher('rightwall/wall2', PolygonStamped, queue_size=1)
pub_rightwall_endpoint_2 = rospy.Publisher('rightwall/endpoint2', PointStamped, queue_size=1)

pub_leftwall = rospy.Publisher('leftwall/wall', PolygonStamped, queue_size=1)
pub_leftwall_endpoint = rospy.Publisher('leftwall/endpoint', PointStamped, queue_size=1)


def get_distance_at_angle(laser_data, angle_deg):
    index_float = np.interp(angle_deg, [math.degrees(laser_data.angle_min), math.degrees(laser_data.angle_max)],
                            [0, len(laser_data.ranges) - 1])
    index = int(index_float)
    distance_to_lidar = laser_data.ranges[index]
    return distance_to_lidar


def get_lidar_point_at_angle(laser_data, angle_deg):
    index_float = np.interp(angle_deg, [math.degrees(laser_data.angle_min), math.degrees(laser_data.angle_max)],
                            [0, len(laser_data.ranges) - 1])
    index = int(index_float)
    distance_to_lidar = laser_data.ranges[index]

    angle_rad = math.radians(angle_deg)
    x = distance_to_lidar * math.cos(angle_rad)
    y = distance_to_lidar * math.sin(angle_rad)

    return np.array([x, y])


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


def get_rightwall_endpoint(laser_data):
    p1 = get_lidar_point_at_angle(laser_data, -90)
    p2 = get_lidar_point_at_angle(laser_data, -90+20)
    delta_y = p2[1] - p1[1]
    delta_x = p2[0] - p1[0]
    denominator = math.sqrt(delta_y**2 + delta_x**2)
    numerator_const_term = p2[0]*p1[1] - p2[1]*p1[0]

    wall_at_infinity = p1 + (p2 - p1)*40
    rightwall = PolygonStamped()
    rightwall.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    rightwall.polygon.points = [Point32(x=p1[0], y=p1[1]),
                                Point32(x=wall_at_infinity[0], y=wall_at_infinity[1])]
    pub_rightwall.publish(rightwall)

    angle = -90 + 20
    endpoint = get_lidar_point_at_angle(laser_data, angle)
    for i in range(20):
        angle += 5
        candid = get_lidar_point_at_angle(laser_data, angle)
        distance = abs(delta_y*candid[0] - delta_x*candid[1] + numerator_const_term)/denominator
        if distance < 0.5:
            endpoint = candid
        else:
            break

    endpoint_msg = PointStamped()
    endpoint_msg.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    endpoint_msg.point = Point(endpoint[0], endpoint[1], 0)
    pub_rightwall_endpoint.publish(endpoint_msg)

    return endpoint


def get_second_rightwall_endpoint(laser_data, wall_angle_start):
    p1 = get_lidar_point_at_angle(laser_data, math.degrees(wall_angle_start))
    p2 = get_lidar_point_at_angle(laser_data, math.degrees(wall_angle_start)+10)
    delta_y = p2[1] - p1[1]
    delta_x = p2[0] - p1[0]
    denominator = math.sqrt(delta_y**2 + delta_x**2)
    numerator_const_term = p2[0]*p1[1] - p2[1]*p1[0]

    wall_at_infinity = p1 + (p2 - p1)*40
    rightwall = PolygonStamped()
    rightwall.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    rightwall.polygon.points = [Point32(x=p1[0], y=p1[1]),
                                Point32(x=wall_at_infinity[0], y=wall_at_infinity[1])]
    pub_rightwall_2.publish(rightwall)

    angle = math.degrees(wall_angle_start)+10
    endpoint = get_lidar_point_at_angle(laser_data, angle)
    for i in range(20):
        angle += 5
        candid = get_lidar_point_at_angle(laser_data, angle)
        distance = abs(delta_y*candid[0] - delta_x*candid[1] + numerator_const_term)/denominator
        if distance < 0.5:
            endpoint = candid
        else:
            break

    endpoint_msg = PointStamped()
    endpoint_msg.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    endpoint_msg.point = Point(endpoint[0], endpoint[1], 0)
    pub_rightwall_endpoint_2.publish(endpoint_msg)

    return endpoint


def get_leftwall_endpoint(laser_data):
    p1 = get_lidar_point_at_angle(laser_data, 90)
    p2 = get_lidar_point_at_angle(laser_data, 90-20)
    delta_y = p2[1] - p1[1]
    delta_x = p2[0] - p1[0]
    denominator = math.sqrt(delta_y**2 + delta_x**2)
    numerator_const_term = p2[0]*p1[1] - p2[1]*p1[0]

    wall_at_infinity = p1 + (p2 - p1)*40
    leftwall = PolygonStamped()
    leftwall.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    leftwall.polygon.points = [Point32(x=p1[0], y=p1[1]),
                                Point32(x=wall_at_infinity[0], y=wall_at_infinity[1])]
    pub_leftwall.publish(leftwall)

    angle = 90 - 20
    endpoint = get_lidar_point_at_angle(laser_data, angle)
    for i in range(20):
        angle -= 5
        candid = get_lidar_point_at_angle(laser_data, angle)
        distance = abs(delta_y*candid[0] - delta_x*candid[1] + numerator_const_term)/denominator
        if distance < 0.5:
            endpoint = candid
        else:
            break

    endpoint_msg = PointStamped()
    endpoint_msg.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    endpoint_msg.point = Point(endpoint[0], endpoint[1], 0)
    pub_leftwall_endpoint.publish(endpoint_msg)

    return endpoint
