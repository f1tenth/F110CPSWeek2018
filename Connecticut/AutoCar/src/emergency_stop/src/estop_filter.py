#!/usr/bin/env python

import rospy
from ac_msgs.msg import drive_params, drive_values
from std_msgs.msg import Bool

# function to map from one range to another, similar to arduino
def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

class StateMaintainer:
    """
    This class maintains the state of the emergency flag. It is needed to ensure
    that the emergency flags are seen by the callback loop.
    """
    def __init__(self):
        self.eflag = False
    

def drive_callback(data, state):
    if(not state.eflag):
        velocity = data.velocity
        angle = data.angle
    else:
        velocity = 0
        angle = 0
        print("Emergency Stop Activated")
    print("Velocity: ", velocity, "Angle: ", angle)
    # Do the computation
    pwm1 = arduino_map(velocity, -100, 100, 6554, 13108);
    pwm2 = arduino_map(angle, -100, 100, 6554, 13108);
    msg = drive_values()
    msg.pwm_drive = pwm1
    msg.pwm_angle = pwm2
    pub.publish(msg)

def eStop_callback(data, state):
    state.eflag=data

# Main function
if __name__ == '__main__':
    print("Emergency Stop Layer Initialized")
    rospy.init_node('emergency_stop_filter')
    state = StateMaintainer()
    pub = rospy.Publisher('drive_pwm', drive_values, queue_size=10)
    em_sub = rospy.Subscriber('eStop', Bool, eStop_callback, state)
    dr_param_sub = rospy.Subscriber("drive_parameters", drive_params, drive_callback, state)
    rospy.spin()
