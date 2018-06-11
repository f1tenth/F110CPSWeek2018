#!/usr/bin/env python

import rospy
from race.msg import drive_param
from race.msg import drive_values

from time import time

delay=[]

def param_recv(data):
        global delay
        delay = time()
        print "param publish"

def pwm_recv(data):
        global delay
        delay = time() - delay
        print "pwm delay: ", delay

if __name__ == '__main__':
	rospy.init_node('time_delay')
	rospy.Subscriber("drive_parameters", drive_param, param_recv)
        rospy.Subscriber("drive_pwm", drive_values, pwm_recv)
	rospy.spin()
