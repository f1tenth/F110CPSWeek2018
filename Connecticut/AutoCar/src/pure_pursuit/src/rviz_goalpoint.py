#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('goal_point_publisher', anonymous=True)
    publisher = rospy.Publisher('/goal_point', PointStamped, queue_size=1)
    while True:
        header = Header(stamp=rospy.Time.now(), frame_id="laser")
        point = Point(1, 1, 1)
        point_stamped = PointStamped(header=header, point=point)

        publisher.publish(point_stamped)


