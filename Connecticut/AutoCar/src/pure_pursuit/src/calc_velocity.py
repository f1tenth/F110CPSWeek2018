import time
import math

prev_time = time.clock()
prev_goalpoint = [0, 0]
velocity = 0


@DeprecationWarning
def calculate_velocity(goalpoint_at_infinity, prev_time, velocity, prev_goalpoint):
    """

    :param goalpoint_at_infinity:
    :param prev_time:
    :param velocity:
    :param prev_goalpoint:
    :return: Velocity, prev_time, prev_goalpoint
    """
    current_time = time.clock()
    delta_time_seconds = current_time - prev_time

    distance = math.sqrt((goalpoint_at_infinity[0] - prev_goalpoint[0])**2 +
                         (goalpoint_at_infinity[1] - prev_goalpoint[1])**2)

    averaging_coeff = 0.8
    velocity = velocity*averaging_coeff + distance/delta_time_seconds*(1-averaging_coeff)

    # TODO Modify return to update these values
    prev_goalpoint = goalpoint_at_infinity
    prev_time = current_time
    # Return is of
    return velocity, prev_time, prev_goalpoint

