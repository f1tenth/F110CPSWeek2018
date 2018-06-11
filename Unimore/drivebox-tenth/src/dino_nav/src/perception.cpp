#include <iostream>
#include <math.h>

#include "perception.h"

#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif

void perception(dinonav_t &nav, const sensor_msgs::LaserScan::ConstPtr& msg) {

    discretize_laserscan(nav.grid, nav.view, nav.conf, nav, msg);

    nav.car_pos.x = nav.grid.size/2;
    nav.car_pos.y = nav.grid.size - nav.conf.ahead_offset/nav.view.cell_l - (nav.car.length*nav.conf.lidar_pos)/nav.view.cell_l;
    inflate(nav.grid, nav.car_pos.x, nav.car_pos.y, 0, 3);
    
}

void discretize_laserscan(grid_t &grid, view_t &view, conf_t &conf, dinonav_t &nav, const sensor_msgs::LaserScan::ConstPtr& msg) {

    float compensate_speed = nav.estimated_speed*0.025f; 

    int quad_l = conf.zoom*2;
    int size = msg->ranges.size();
    double angle = msg->angle_max + M_PI*3/2;

    float noise_toll = 0.10;

    for(int i=0; i<size; i++) {
        float r = msg->ranges[i];
        
        bool evaluate = false;
        float  r_prec, r_succ;
        i>0 ?       r_prec = msg->ranges[i-1] : r_prec = r;
        i<size-1 ?  r_succ = msg->ranges[i+1] : r_prec = r;
        if(fabs(r - r_prec) < noise_toll || fabs(r - r_succ) < noise_toll)
            evaluate = true;

        if(i==size/2)
            grid.middle_id = grid.points_n;

        if(evaluate) {
            //quad_l : view_l = r : view_r
            //coodianates of the sigle ray
            float view_r = r*view.l/quad_l; 
            float x = view.l/2 + cos(angle) * view_r;
            float y = view.l -conf.ahead_offset + sin(angle) * view_r + meters2view(nav, compensate_speed);

            //coordinates of the corrispondent cell
            int grid_x = x / view.cell_l;
            int grid_y = y / view.cell_l;
            if(setgrid(grid, grid_x, grid_y, WALL)) {

                int n = grid.points_n;
                if( n == 0 || ( n>0                            &&
                                grid.points[n-1].x != grid_x   ||
                                grid.points[n-1].y != grid_y       )) {

                    grid.points[n].x = grid_x;
                    grid.points[n].y = grid_y;
                    grid.points_n++;
                }
            }
            inflate(grid, grid_x, grid_y, INFLATED, conf.inflation);
        }
        //if(i>0 && (last_x != grid_x || last_y != grid_y)) {
        //    grid_line(grid, grid_x, grid_y, last_x, last_y, GATE);
        //}
        angle -= msg->angle_increment;
    }

    for(int i=1; i<grid.points_n; i++) {
        point_t p = grid.points[i];
        point_t prev = grid.points[i-1];
        if(prev.x != p.x || prev.y != p.y) {
            grid_line(grid, p.x, p.y, prev.x, prev.y, GATE);
        }
    }
}
