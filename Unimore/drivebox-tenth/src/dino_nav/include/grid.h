#ifndef GRID_H
#define GRID_H

#include "common.h"

#define GRID_MAX_DIM 1000
#define GRID_MAX_GATES 64

struct gate_t {
    point_t s, e;
    int dim;
};

struct grid_t {

    int *data;
    int size;

    point_t points[GRID_MAX_DIM*2];
    int middle_id;
    int points_n;

    gate_t gates[GRID_MAX_GATES];
    int gates_n;
};

#include "dinonav.h"

void init_grid(grid_t &grid, int size);

int grid_line(grid_t &grid, int x1, int y1, int x2, int y2, int value);
bool grid_line_control(grid_t &grid, int x1, int y1, int x2, int y2);

bool setgrid(grid_t &grid, int x, int y, int value);
int getgrid(grid_t &grid, int x, int y);

void inflate(grid_t &grid, int cx, int cy, int val, int radius);

float_point_t grid2view(int x, int y, view_t &view);
point_t view2grid(float x, float y, view_t &view);

#endif //GRID_H
