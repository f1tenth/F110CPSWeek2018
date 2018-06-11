#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped

old_pos = 0
old_time = 0
speed = 0


def callback(data):
	global old_pos, old_time, speed

	pos = data.pose.position
	if(old_pos == 0):
		old_pos = pos

	time = data.header.stamp
	if(old_time == 0):
		old_time = time

	dx = pos.x - old_pos.x
	dy = pos.y - old_pos.y
	dst = math.sqrt(dx*dx + dy*dy)	
	
	dt = (time - old_time).to_sec()
	print dst/dt

	old_pos = pos
	old_time = time

if __name__ == '__main__':
	print("Speed estimation initialized")
	rospy.init_node('speed_calc')
	rospy.Subscriber("pose_stamped", PoseStamped, callback)
	rospy.spin()
