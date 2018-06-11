//
// Created by cecco on 07/12/16.
//
#include <math.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "grid.h"
#include "dinonav.h"


void init_grid(grid_t &grid, int size) {

    grid.size = size;

    //init grid
    for(int i=0; i<size*size; i++)
        grid.data[i] = EMPTY;
    grid.gates_n = 0;
    grid.points_n = 0;
}


/**
    geterate a line in a matrix from point 1 to 2
*/
int grid_line(grid_t &grid, int x1, int y1, int x2, int y2, int value) {
    //gate search
    int startGx = -1, startGy = -1;
    int endGx = -1,   endGy = -1;

    int dx = x1 - x2;
    int dy = y1 - y2;
    int steps;

    if (abs(dx) > abs(dy))
        steps = abs(dx);
    else
        steps = abs(dy);

    float x_inc = -(float)dx / (float) steps;
    float y_inc = -(float)dy / (float) steps;

    if(steps > grid.size)
        return 0;

    float x = x1, y = y1;
    for(int v=0; v < steps; v++)
    {
        x = x + x_inc;
        y = y + y_inc;
        int xx = x, yy = y;
        if((value == 0 &&  getgrid(grid, xx, yy) == GATE) || getgrid(grid, xx, yy) < value) {
            setgrid(grid, xx, yy, value);
            if(startGx == -1) {
                startGx = xx; startGy = yy;
            } else {
                endGx = xx; endGy = yy;
            }
        }
    }

    if(grid.gates_n >= GRID_MAX_GATES) {
        ROS_WARN("Reached max gates");
        return 0;
    }

    if(startGx != -1 && endGx != -1) {
        //startGx and endGx are effective gate bounds 
        gate_t *g = &grid.gates[grid.gates_n];
        g->s.x = x1;
        g->s.y = y1;
        g->e.x = x2;
        g->e.y = y2;
        g->dim = steps;
        grid.gates_n++;
        //printf("start: %d %d, end: %d %d\n", startGx, startGy, endGx, endGy);
    }
    return steps;
}

/**
    Control if a line in a Matrix is possible to draw without using
    filled cells.
*/
bool grid_line_control(grid_t &grid, int x1, int y1, int x2, int y2) {

    int dx = x1 - x2;
    int dy = y1 - y2;
    int steps;

    if (abs(dx) > abs(dy))
        steps = abs(dx);
    else
        steps = abs(dy);

    float x_inc = -(float)dx / (float) steps;
    float y_inc = -(float)dy / (float) steps;

    float x = x1, y = y1;
    for(int v=0; v < steps; v++)
    {
        x = x + x_inc;
        y = y + y_inc;
        int xx = x, yy = y;
        int val = getgrid(grid, xx, yy);
        int val1 = getgrid(grid, xx + 1, yy);
        int val2 = getgrid(grid, xx - 1, yy);
        if(val != EMPTY || val1 != EMPTY || val2 != EMPTY)
            return false;

        //setgrid(grid, xx +1, yy, 10);
        //setgrid(grid, xx -1, yy, 10);
        //setgrid(grid, xx, yy, 10);
    }
    return true;
}


/**
    set a grid to value in given position
*/
bool setgrid(grid_t &grid, int x, int y, int value) {
    int pos = y*grid.size + x;

    if(x<0 || x >= grid.size || y<0 || y >= grid.size)
        return false;
    
    int old = grid.data[pos];
    if( (value == EMPTY && old != WALL) || old < value)
        grid.data[pos] = value;
    return true;
}

/**
    get a grid to value in given position
*/
int getgrid(grid_t &grid, int x, int y) {
    int pos = y*grid.size + x;

    if(x<0 || x >= grid.size || y<0 || y >= grid.size)
        return -1;
    return grid.data[pos];
}

/**
    inflate a point with the given spread
            X 
    X ->  X X X   example with n = 1
            X 
*/
void inflate(grid_t &grid, int cx, int cy, int val, int radius) { 

    for(int y=-radius; y<=radius; y++)
        for(int x=-radius; x<=radius; x++)
            if(x*x+y*y <= radius*radius)
                setgrid(grid, cx+x, cy+y, val);
}

/**
    convert matrix coords to screen position
*/
float_point_t grid2view(int x, int y, view_t &view) {
    float_point_t p;
    p.x = view.x + x*view.cell_l + view.cell_l/2;
    p.y = view.y + y*view.cell_l + view.cell_l/2;
    return p;
}

/**
    convert sceen position to matrix coords
*/
point_t view2grid(float x, float y, view_t &view) {
    point_t p;
    p.x = (x - view.x)/view.cell_l;
    p.y = (y - view.y)/view.cell_l;

    return p;
}

/**
    convert view dist to meters
*/
float view2meters(dinonav_t &nav, float view_dist) {

    return view_dist*((nav.conf.zoom*2)/nav.view.l);
}

/**
    convert view dist to meters
*/
float meters2view(dinonav_t &nav, float mtr_dist) {

    return mtr_dist/((nav.conf.zoom*2)/nav.view.l);
}