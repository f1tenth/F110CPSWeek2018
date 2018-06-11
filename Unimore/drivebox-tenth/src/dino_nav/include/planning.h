#ifndef PLANNING_H
#define PLANNING_H

#include "common.h"
#include "grid.h"
#include "dinonav.h"

void planning(dinonav_t &nav);

int choosegate(grid_t &grid, int px, int py);

segment_t calc_curve(grid_t &grid, int gate_idx, float_point_t start,
    view_t &view, car_t &car, track_t &track, conf_t &conf);

#endif //PLANNING_H