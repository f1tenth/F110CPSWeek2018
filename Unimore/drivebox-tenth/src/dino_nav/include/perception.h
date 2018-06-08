#ifndef PERCEPTION_H
#define PERCEPTION_H

#include "common.h"
#include "grid.h"
#include "dinonav.h"

void perception(dinonav_t &nav, const sensor_msgs::LaserScan::ConstPtr& msg);

void discretize_laserscan(grid_t &grid, view_t &view, conf_t &conf, dinonav_t &nav, const sensor_msgs::LaserScan::ConstPtr& msg);

#endif //PERCEPTION_H