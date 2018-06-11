#ifndef ACTUATION_H
#define ACTUATION_H

#include "ackermann_msgs/AckermannDrive.h"
#include "dinonav.h"

void actuation(dinonav_t &nav, ackermann_msgs::AckermannDrive &drive_msg);

float calc_steer(float_point_t &start, view_t &view, car_t &car, grid_t &grid, path_t &path, int &steer_l);
float calc_throttle(conf_t &conf, view_t &view, car_t &car, track_t &track, segment_t &curve, 
    float curve_dst, float estimated_speed, float estimated_acc, float &target_acc);

#endif //ACTUATION_H