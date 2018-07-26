from lidar_utils import *

import rospy
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped, Point
from std_msgs.msg import Header

pub_poly1 = rospy.Publisher('polygons/p1', PolygonStamped, queue_size=1)
pub_poly2 = rospy.Publisher('polygons/p2', PolygonStamped, queue_size=1)
pub_point1 = rospy.Publisher('center/p1', PointStamped, queue_size=1)
pub_point2 = rospy.Publisher('center/p2', PointStamped, queue_size=1)


def calculate_goalpoint_distance_from_wall(laser_data, distance):
    p1 = get_lidar_point_at_angle(laser_data, -90)
    p2 = get_lidar_point_at_angle(laser_data, -80)
    p3 = get_lidar_point_at_angle(laser_data, -70)

    def calc_point(p1, p2, d):
        mp = (p1+p2)/2
        slopeinv = (p1[0]-p2[0])/(p1[1]-p2[1])
        dx = abs(np.sqrt((d**2)/(1+(1/slopeinv**2))))
        dy = abs(np.sqrt((d**2)/(1+(slopeinv**2))))
        rp = np.array([-1*(mp[0]-dx), (mp[1]-dy)])
        # print("GP Details: P1= "+str(p1), "P2="+str(p2), "MP= "+str(mp), "Dx= "+str(dx), "Dy= "+str(dy), "RP= "+str(rp))
        return rp

    gp1 = calc_point(p1, p2, distance)
    gp2 = calc_point(p2, p3, distance)

    # Publish for visualization

    center = PointStamped()
    center.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    center.point = Point(gp1[0], gp1[1], 0)
    pub_point1.publish(center)
    center.point = Point(gp2[0], gp2[1], 0)
    pub_point2.publish(center)
    
    return (gp1 + gp2)/2


def calculate_goalpoint_manhattan(laser_data):
    right_end = lidar_to_rear_axle_coords(get_rightwall_endpoint(laser_data))
    left_end = lidar_to_rear_axle_coords(get_leftwall_endpoint(laser_data))
    goalpoint_at_infinity = (right_end + left_end) / 2
    return goalpoint_at_infinity


def calculate_goalpoint_double_manhattan(laser_data):
    right_end = lidar_to_rear_axle_coords(get_rightwall_endpoint(laser_data))
    right_end_angle_rad = math.atan(right_end[1] / right_end[0])
    right_end_2 = lidar_to_rear_axle_coords(get_second_rightwall_endpoint(laser_data, right_end_angle_rad))
    right_end_2 *= .4
    left_end = lidar_to_rear_axle_coords(get_leftwall_endpoint(laser_data))
    goalpoint_at_infinity = (right_end + right_end_2 + left_end) / 2.4
    return goalpoint_at_infinity


def calculate_goalpoint_two_boxes(laser_data):
    poly = PolygonStamped()
    center = PointStamped()

    p1 = get_lidar_point_at_angle(laser_data, -80)
    p2 = get_lidar_point_at_angle(laser_data, -70)
    p3 = get_lidar_point_at_angle(laser_data, 70)
    p4 = get_lidar_point_at_angle(laser_data, 80)

    poly.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    poly.polygon.points = [Point32(x=p1[0], y=p1[1]),
                           Point32(x=p2[0], y=p2[1]),
                           Point32(x=p3[0], y=p3[1]),
                           Point32(x=p4[0], y=p4[1])]
    pub_poly1.publish(poly)

    center1 = (p1 + p2 + p3 + p4) / 4
    center.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    center.point = Point(center1[0], center1[1], 0)
    pub_point1.publish(center)

    p5 = get_lidar_point_at_angle(laser_data, -50)
    p6 = get_lidar_point_at_angle(laser_data, -40)
    p7 = get_lidar_point_at_angle(laser_data, 40)
    p8 = get_lidar_point_at_angle(laser_data, 50)

    poly.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    poly.polygon.points = [Point32(x=p5[0], y=p5[1]),
                           Point32(x=p6[0], y=p6[1]),
                           Point32(x=p7[0], y=p7[1]),
                           Point32(x=p8[0], y=p8[1])]
    pub_poly2.publish(poly)

    center2 = (p5 + p6 + p7 + p8) / 4
    center.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    center.point = Point(center2[0], center2[1], 0)
    pub_point2.publish(center)

    centerboxes = (center1+center2)/2

    return lidar_to_rear_axle_coords(centerboxes)


def calculate_goalpoint_one_box(laser_data):
    p0 = get_lidar_point_at_angle(laser_data, -80)
    p1 = get_lidar_point_at_angle(laser_data, -70)
    p2 = get_lidar_point_at_angle(laser_data, 70)
    p3 = get_lidar_point_at_angle(laser_data, 80)

    poly = PolygonStamped()
    poly.header = Header(stamp=rospy.Time.now(), frame_id="laser")
    poly.polygon.points = [Point32(x=p0[0], y=p0[1]),
                           Point32(x=p1[0], y=p1[1]),
                           Point32(x=p2[0], y=p2[1]),
                           Point32(x=p3[0], y=p3[1])]
    pub_poly1.publish(poly)

    p = (p0 + p1 + p2 + p3) / 4
    return lidar_to_rear_axle_coords(p)


def calculate_goalpoint2(laser_data):
    p1 = lidar_to_rear_axle_coords(get_lidar_point_at_angle(laser_data, -80))
    p2 = lidar_to_rear_axle_coords(get_lidar_point_at_angle(laser_data, 80))
    p = (p1 + p2)/2
    return p


def calculate_goalpoint1(laser_data):
    p = {}
    for i in range(21):
        p[i] = get_distance_at_angle(laser_data, np.interp(i, [0, 20], [-90, 90]))

    diff = {}
    for j in range(21):
        diff[j] = abs(p[j] - p[20-j])

    maxindex = 0
    maxval = -10000

    for j in range(10) :
        if diff[j] >= maxval :
            maxindex = j
            maxval = diff[j]

    p1 = lidar_to_rear_axle_coords(get_lidar_point_at_angle(laser_data, np.interp(maxindex, [0, 20], [-90, 90])))
    p2 = lidar_to_rear_axle_coords(get_lidar_point_at_angle(laser_data, -1*np.interp(maxindex, [0, 20], [-90, 90])))
    p = (p1 + p2)/2
    print("p1: ", p1)
    print("p2: ", p2)
    print("p: ", p)

    return p
