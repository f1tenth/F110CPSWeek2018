#!/usr/bin/env python

import rospy
from race.msg import drive_param
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates
import tf
'''
this node get the drive parameters from the keyboard node and convert it in
AckermannDrive for the simulator

'''
pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=1)

def control(data):
        
        input_vel = data.velocity
        input_angle = data.angle

        if input_vel >  100: input_vel =  100
        if input_vel < -100: input_vel = -100

        if input_angle >  100: input_angle =  100
        if input_angle < -100: input_angle = -100

        speed = input_vel / 50
        angle = -input_angle/100*0.8
        print "vel: ", input_vel, "->", speed, "\t\tangle:", input_angle, "->", angle

        msg = AckermannDrive();
        msg.speed = speed
        msg.steering_angle = angle
        pub.publish(msg)
        

def tf_pub(data):
        if "ackermann_vehicle" in data.name: 
                pose = data.pose[1]
                br = tf.TransformBroadcaster()
                p = (pose.position.x, pose.position.y, pose.position.z)
                o = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
                br.sendTransform(p, o, rospy.Time.now(), "footprint", "map")		

if __name__ == '__main__':
	rospy.init_node('sim_controller')
	rospy.Subscriber("drive_parameters", drive_param, control)
	rospy.Subscriber("/gazebo/model_states", ModelStates, tf_pub)
	rospy.spin()
