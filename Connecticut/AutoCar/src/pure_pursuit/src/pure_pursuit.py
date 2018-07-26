import math
import numpy as np

def pure_pursuit(goalpoint_angle_rad, distance_to_goalpoint=0.6, wheelbase=0.33, max_turn_degrees=34):
    # goalpoint_angle_rad is alpha in CMU's formulation
    # goalpoint_angle_rad must be in (-pi/2, pi/2)
    # wheelbase is the distance (in meters) between the rear and front axels
    # distance_to_goalpoint is the distance (in meters) between the rear axel and the goal point
    # distance_to_goalpoint must be positive

    steering_angle_rad = math.atan(2 * wheelbase * math.sin(goalpoint_angle_rad) / distance_to_goalpoint)
    steering_angle_deg = math.degrees(steering_angle_rad)

    # The wheels cannot physically turn more than 34 degrees
    if steering_angle_deg < -max_turn_degrees:
        steering_angle_deg = -max_turn_degrees
    elif steering_angle_deg > max_turn_degrees:
        steering_angle_deg = max_turn_degrees

    # The steering command is in percentages
    steering_percentage = np.interp(steering_angle_deg, [-max_turn_degrees, max_turn_degrees], [-100, 100])
    return int(steering_percentage)
