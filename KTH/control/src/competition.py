#!/usr/bin/env python
"""u 
/*
 * 
 * 
 * 
 *   
    Author: Xiao Chen
    Royal Institute of Technology, Stockholm 2018
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *
 *
 *
 */
"""
import os
import rospy
import tf
import tf2_ros
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from control.msg import pwm_interface
from control.srv import SetData

class reactive_control():
    # input parameters:
    #   center_to_front: distance from gravitation center(base_link) to front axel
    #   center_to_back:  distance from gravitation center(base_link) to back axel
    #   lidar_distance:  distance from lidar to gravitation center(base_link)
    def __init__(self, center_to_front = 0.18, center_to_back = 0.15, lidar_distance = 0.25):
        #self.speed_choose = rospy.get_param('~speed', 1)


        #####################################
        # vehicle geometrical properties    #
        #####################################
        self.axel_length = center_to_front + center_to_back
        self.center_to_front = center_to_front
        self.center_to_back = center_to_back
        self.lidar_distance = lidar_distance        

        #####################################
        #PWM control signal parameters      #
        #####################################
        self.PWM_VEL_MIN = 0       #minimum speed pwm
        self.PWM_VEL_MAX = 30      #maximum speed pwm

        self.STEER_MAX = 40        #maximum steering angle
        self.SAT_STEER_MIN = -81   #minimum steering pwm
        self.SAT_STEER_MAX = 81    #maximum steering pwm
    
        self.PWM_STEER = 0         #actual steering command
        self.PWM_VEL = 0           #actual velocity command

        #####################################
        #set up rosnode and interface       #
        #####################################
        rospy.init_node("reactive_race_node")
        rospy.Subscriber("scan", LaserScan, self.control_step)

        self.pub_pwm = rospy.Publisher("pwm_interface", pwm_interface, queue_size = 1)
        #publisher for scan average debugging
        self.scan_pub = rospy.Publisher("scan_average", LaserScan, queue_size = 1)
        #publisher for target point debuggning
        self.pub_target = rospy.Publisher("target_pose", Odometry, queue_size = 1)
        self.pub_right = rospy.Publisher("right_pose", Odometry, queue_size = 1)
        self.pub_left = rospy.Publisher("left_pose", Odometry, queue_size = 1)
        self.pub_ahead = rospy.Publisher("ahead_pose", Odometry, queue_size = 1)
        self.pub_right_ahead = rospy.Publisher("ahead_right_pose", Odometry, queue_size = 1)
        self.pub_left_ahead = rospy.Publisher("ahead_left_pose", Odometry, queue_size = 1)

        #service for manually set speed and lookahead
        rospy.Service('set_speed', SetData, self.handle_set_speed)
        rospy.Service('set_lookahead', SetData, self.handle_set_lookahead_dist)

        rospy.spin()    

    def control_step(self, scan_msg):
        read_density = 4
        scan_reading_average = [0]*1081

        ###########################################
        #speed adjusting based on aheaddistance   #
        ###########################################

        ahead_distance = scan_msg.ranges[540]
        #if self.speed_choose == 2:
            #rospy.loginfo('sppppppppppppppppppppppppp : ')

        if ahead_distance >= 5:
            self.PWM_VEL = 25
            self.lookahead_distance = 0.9
        elif ahead_distance >= 3.5 and ahead_distance <= 5:
            self.PWM_VEL = 15
            self.lookahead_distance = 0.6
        else:
            self.PWM_VEL = 10
            self.lookahead_distance = 0.4        


        scan_reading_average_left = []
        scan_reading_angle_left = []
        scan_reading_average_right = []
        scan_reading_angle_right = []        

        ######################################
        # scan data smoothing                #
        ######################################
        average_window_right = 10
        average_window_left = 10
        right_limit = 540 - 30 * 4
        left_limit = 540 + 45 * 4                

        #righthand side smoothing from -135+10 to -30
        for i in range(0 + read_density * average_window_right, right_limit, read_density):
            scan = 0
            for j in range(-average_window_right, average_window_right + 1): 
                    scan += scan_msg.ranges[i + read_density*j]
            scan_reading_average[i] = (scan - scan_msg.ranges[i])/(2*average_window_right)
            scan_reading_average_right.append((scan - scan_msg.ranges[i])/(2*average_window_right))
            scan_reading_angle_right.append(i*0.25 - 135) 

        #lefthand side smoothing from 45 to 135-10
        for i in range(left_limit, 1080 - read_density * average_window_left - read_density * 15, read_density):
            scan = 0
            for j in range(-average_window_left, average_window_left + 1):       
                    scan += scan_msg.ranges[i + read_density*j]
            scan_reading_average[i] = (scan - scan_msg.ranges[i])/(2*average_window_left)
            scan_reading_average_left.append((scan - scan_msg.ranges[i])/(2*average_window_left))
            scan_reading_angle_left.append((i-540)*0.25)          

        ######################################
        # publishing the smoothed laser      #
        ######################################
        laser_average_msg = LaserScan()
        laser_average_msg = scan_msg
        laser_average_msg.ranges = scan_reading_average
        self.scan_pub.publish(laser_average_msg)   

        ######################################
        # search for shortest distance point #
        ######################################

        #search for the right hand point start from -135+10 to -30
        right_min_distance = 1000
        for i in range(0, len(scan_reading_average_right)):
            if scan_reading_average_right[i] <= right_min_distance:
                right_min_distance = scan_reading_average_right[i]
                right_min_angle = scan_reading_angle_right[i]*3.14/180
                right_min_index = i

        vector_right_x = self.lidar_distance + right_min_distance * np.cos(right_min_angle)
        vector_right_y = right_min_distance * np.sin(right_min_angle)
        vector_right_length = np.sqrt(vector_right_x**2 + vector_right_y**2)
        rospy.loginfo('right_min_angle : ' + str(right_min_angle*180/3.14))

        #seaarch for the left hand point start from 75 to 135-10
        left_min_distance = 1000
        for i in range(25, len(scan_reading_average_left)):
            if scan_reading_average_left[i] <= left_min_distance:
                left_min_distance = scan_reading_average_left[i]
                left_min_angle = scan_reading_angle_left[i]*3.14/180
                left_min_index = i

        vector_left_x = self.lidar_distance + left_min_distance * np.cos(left_min_angle)
        vector_left_y = left_min_distance * np.sin(left_min_angle)
        vector_left_length = np.sqrt(vector_left_x**2 + vector_left_y**2)
        rospy.loginfo('left_min_angle : ' + str(left_min_angle*180/3.14))

        lookahead_angles = 28
        lookahead_left_margin = abs(left_min_index - max((left_min_index - lookahead_angles), 0))
        lookahead_right_margin = abs(min((right_min_index + lookahead_angles), len(scan_reading_average_right)) - right_min_index)

        ahead_left_length = np.cos(lookahead_left_margin*3.14/180) * scan_reading_average_left[max((left_min_index - lookahead_angles), 0)]
        ahead_right_length = np.cos(lookahead_right_margin*3.14/180) * scan_reading_average_right[min((right_min_index + lookahead_angles), len(scan_reading_average_right)-1)]
        
        ahead_left_x = ahead_left_length * vector_left_x / vector_left_length
        ahead_left_y = ahead_left_length * vector_left_y / vector_left_length
        ahead_right_x = ahead_right_length * vector_right_x / vector_right_length
        ahead_right_y = ahead_right_length * vector_right_y / vector_right_length
        
        ahead_difference_x = ahead_right_x - ahead_left_x
        ahead_difference_y = ahead_right_y - ahead_left_y
        ahead_difference_length = np.sqrt(ahead_difference_x**2 + ahead_difference_y**2)
        ahead_difference_x = ahead_difference_x / ahead_difference_length
        ahead_difference_y = ahead_difference_y / ahead_difference_length
        ahead_vector_x = ahead_left_x + 0.5 * ahead_difference_length * ahead_difference_x
        ahead_vector_y = ahead_left_y + 0.5 * ahead_difference_length * ahead_difference_y
        ahead_vector_x = ahead_vector_x + self.lookahead_distance * (-ahead_difference_y)
        ahead_vector_y = ahead_vector_y + self.lookahead_distance * (ahead_difference_x)


        vector_difference_x = vector_right_x - vector_left_x
        vector_difference_y = vector_right_y - vector_left_y
        vector_difference_length = np.sqrt(vector_difference_x**2 + vector_difference_y**2)
        vector_difference_x = vector_difference_x / vector_difference_length
        vector_difference_y = vector_difference_y / vector_difference_length

        target_vector_x = vector_left_x + 0.5 * vector_difference_length * vector_difference_x
        target_vector_y = vector_left_y + 0.5 * vector_difference_length * vector_difference_y
        target_vector_x = target_vector_x + self.lookahead_distance * (-vector_difference_y)
        target_vector_y = target_vector_y + self.lookahead_distance * (vector_difference_x)

        self.odom_publish("target",target_vector_x, target_vector_y)
        self.odom_publish("left", vector_left_x, vector_left_y)
        self.odom_publish("right", vector_right_x, vector_right_y)

        self.odom_publish("ahead", ahead_vector_x, ahead_vector_y)

        self.odom_publish("leftahead", ahead_left_x, ahead_left_y)
        self.odom_publish("rightahead", ahead_right_x, ahead_right_y)

        error = 0.0
        if right_min_distance <= 0.3:
            error = abs(right_min_distance - 0.3)



        distance_to_target = np.sqrt(ahead_vector_x**2 + ahead_vector_y**2)

        self.PWM_STEER = np.arctan(self.axel_length * 2 * ahead_vector_y / distance_to_target ** 2) * 180 / np.pi + 100 * error

        self.PWM_STEER = (self.PWM_STEER) / (self.STEER_MAX) * 100 

        self.send_commands()

    def send_commands(self):
        msg = pwm_interface()
        self.PWM_VEL = min(max(self.PWM_VEL, self.PWM_VEL_MIN), self.PWM_VEL_MAX)
        self.PWM_STEER = -min(max(self.PWM_STEER, self.SAT_STEER_MIN), self.SAT_STEER_MAX)

        msg.velocity = self.PWM_VEL#int(round((3277. / 100.) * self.PWM_VEL) + 9831.)  # will be publishing to velocity ctrl later
        msg.steering = self.PWM_STEER#int(round((3277. / 100.) * self.PWM_STEER) + 9831.)
        self.pub_pwm.publish(msg)

    def handle_set_speed(self, req):
        self.PWM_VEL = int(req.data)
        return True

    def handle_set_lookahead_dist(self, req):
        self.lookahead_distance = req.data
        return True

    def odom_publish(self, option, vector_x, vector_y):
        q = tf.transformations.quaternion_from_euler(0, 0, np.arctan(vector_y/vector_x))
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "base_link"

        odom.pose.pose.position.x = vector_x
        odom.pose.pose.position.y = vector_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        if option == "target":
            odom.child_frame_id = "target_pose_point"
            self.pub_target.publish(odom)
        elif option == "left":
            odom.child_frame_id = "left_pose_point"
            self.pub_left.publish(odom)
        elif option == "right":
            odom.child_frame_id = "right_pose_point"
            self.pub_right.publish(odom)
        elif option == "ahead":
            odom.child_frame_id = "ahead_pose_point"
            self.pub_ahead.publish(odom)
        elif option == "rightahead":
            odom.child_frame_id = "right_pose_ahead"
            self.pub_right_ahead.publish(odom)
        elif option == "leftahead":
            odom.child_frame_id = "left_pose_ahead"
            self.pub_left_ahead.publish(odom)

if __name__ == '__main__':
    race = reactive_control()
