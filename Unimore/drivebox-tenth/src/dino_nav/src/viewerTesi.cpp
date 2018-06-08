#include <iostream>
#include <math.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>

#include "ros/ros.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "std_msgs/Float32.h"
#include "dino_nav/Stat.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include "grid.h"
#include "pathfind.h"
#include "dinonav.h"
#include "common.h"
#include "viewer.h"
#include "viz.h"

extern ALLEGRO_DISPLAY *display;
ALLEGRO_BITMAP *path_bmp = NULL;
ALLEGRO_BITMAP *view_bmp = NULL;

void draw_pose(view_t &view, geometry_msgs::Pose pose) {
    
    view_t v; 
    v.x = view.x + view.l + 10;
    v.y = view.y;
    v.l = view.l;

    if(path_bmp == NULL) {
        path_bmp = al_create_bitmap(v.l, v.l);
        al_set_target_bitmap(path_bmp);
            al_clear_to_color(al_map_rgb(0,0,0));
        al_set_target_bitmap(al_get_backbuffer(display));
    }
    float x = pose.position.x*10;
    float y = pose.position.y*10;

    float v_startx = v.x + v.l/2;
    float v_starty = v.y + v.l/2;
    float v_px = v_startx + x;
    float v_py = v_starty + y;

    al_set_target_bitmap(path_bmp);
        al_put_pixel(v.l/2 +x, v.l/2 +y, PATH_COLOR);
    al_set_target_bitmap(al_get_backbuffer(display));

    al_draw_bitmap(path_bmp, v.x, v.y, 0);
    al_draw_circle(v_startx, v_starty, 2, PATH_COLOR, 1);
    al_draw_circle(v_px, v_py, 2, VIEW_COLOR, 1);

    tf::Quaternion q(   pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    al_draw_line(v_px, v_py, v_px+cos(yaw)*10, v_py+sin(yaw)*10, VIEW_COLOR, 1);
    viz_text(v.x, v.y + v.l +5, 15, VIEW_COLOR, "yaw: %.3f", yaw);

    float_point_t L, R, L2, R2;
    float l = 60;
    L.x = v_px + cos(yaw - M_PI/2)*l/2;
    L.y = v_py + sin(yaw - M_PI/2)*l/2;
    R.x = v_px + cos(yaw + M_PI/2)*l/2;
    R.y = v_py + sin(yaw + M_PI/2)*l/2;
    L2.x = L.x + cos(yaw)*l;
    L2.y = L.y + sin(yaw)*l;
    R2.x = R.x + cos(yaw)*l;
    R2.y = R.y + sin(yaw)*l;

    float scale = l / v.l;
    al_draw_scaled_rotated_bitmap(
        view_bmp, v.l/2, v.l, v_px, v_py, scale, scale, yaw + M_PI/2, 0);

    al_draw_line(v_px, v_py, L.x, L.y, VIEW_COLOR, 1);
    al_draw_line(v_px, v_py, R.x, R.y, VIEW_COLOR, 1);
    al_draw_line(L.x, L.y, L2.x, L2.y, VIEW_COLOR, 1);
    al_draw_line(R.x, R.y, R2.x, R2.y, VIEW_COLOR, 1);
    al_draw_line(L2.x, L2.y, R2.x, R2.y, VIEW_COLOR, 1);
    al_draw_rectangle(v.x, v.y, v.x + v.l, v.y + v.l, VIEW_COLOR, 1);

    /*
    float m_o = (yaw + M_PI/2);
    float q_o = v_py - m_o*v_px; 
    float dst_o = (mouse.y - (m_o*mouse.x +q_o)) / sqrt(1 + m_o*m_o);
    
    float m_v = yaw;
    float q_v = v_py - m_v*v_px; 
    float dst_v = (mouse.y - (m_v*mouse.x +q_v)) / sqrt(1 + m_v*m_v);
    al_draw_circle(view.x + view.l/2 + dst_v/l*view.l, view.y + view.l + dst_o/l*view.l, 2, VIEW_COLOR, 1);
    */
}


float lasers[2000];
int lsize = 0;
int SIZE= 800;
float laser_max = 0;;
float laser_inc = 0;

void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg) {
    lsize = msg->ranges.size();
    laser_max = msg->angle_max;
    laser_inc = msg->angle_increment;
    for(int i=0; i<lsize; i++)
        lasers[i] = msg->ranges[i];
}

void map_recv(const dino_nav::Stat::ConstPtr& msg) {
    ROS_INFO("stat recived");
    al_clear_to_color(RGBA(1, 1, 1, 1.0));

    grid_t grid;
    int grid_dim = msg->grid_size;
    grid.size = grid_dim;
    
    static int *grid_hex = NULL;
    if(grid_hex == NULL)
        grid_hex = new int[GRID_MAX_DIM*GRID_MAX_DIM];
    grid.data = grid_hex;
    
    view_t view;
    view.x = 10; view.y = 10;
    view.l = SIZE;
    view.cell_l = view.l / float(grid_dim);

    for(int i=0; i<grid_dim; i++) {
        for (int j = 0; j < grid_dim; j++)
            grid.data[i*grid_dim +j] = msg->grid[i*grid_dim + j];
    }

    ///////// LASER SCAN ////////
    int quad_l = msg->zoom*2;
    int size = lsize;
    double angle = laser_max + M_PI*3/2;

    //float noise_toll = 0.10;
    float_point_t orig;
    orig.x = view.x + view.l/2;
    orig.y = view.y + view.l;
    viz_circle(orig, 3, RGBA(0,0,0,1), 1);

    for(int i=0; i<size; i++) {
        float r = lasers[i];
        /*  
        bool evaluate = false;
        float  r_prec, r_succ;
        i>0 ?       r_prec = msg->ranges[i-1] : r_prec = r;
        i<size-1 ?  r_succ = msg->ranges[i+1] : r_prec = r;
        if(fabs(r - r_prec) < noise_toll || fabs(r - r_succ) < noise_toll)
            evaluate = true;
        
        if(i==size/2)
            grid.middle_id = grid.points_n;
        */
        if(true) {
            //quad_l : view_l = r : view_r
            //coodianates of the sigle ray
            float view_r = r*view.l/quad_l; 
            float_point_t p;
            p.x = view.x + view.l/2 + cos(angle) * view_r;
            p.y = view.y + view.l + sin(angle) * view_r;

        
            viz_circle(p, 1, RGBA(0,0,0,1), 0);
        }
        //if(i>0 && (last_x != grid_x || last_y != grid_y)) {
        //    grid_line(grid, grid_x, grid_y, last_x, last_y, GATE);
        //}
        angle -= laser_inc;
    }
    ////////////////////////////////////////////////////////////////////////


   if(view_bmp == NULL) {
        view_bmp = al_create_bitmap(view.l, view.l);
    }
    al_set_target_bitmap(view_bmp);
    al_clear_to_color(al_map_rgba(0,0,0,0));

    for(int i=0; i<grid_dim; i++) {
        for (int j = 0; j < grid_dim; j++) {
            int val = grid.data[i*grid_dim +j];
            ALLEGRO_COLOR col;

            if (val == WALL) {
                col = RGBA(0,0,0,1);
            } else if(val == INFLATED) {
                col =  RGBA(0,0,0,0.5);  
            } else if(val == GATE) {
                col =  RGBA(0,0,0,0);  
            } else if(val == PATH) {
                col =  RGBA(0,0,0,0);
            } else {
                continue;
            }

            //al_draw_filled_rectangle(   view.cell_l * j, view.cell_l * i, view.cell_l * (j + 1),
            //                            view.cell_l * (i + 1), col);
        }
    }
    al_set_target_bitmap(al_get_backbuffer(display));
    al_draw_bitmap(view_bmp, view.x, view.y, 0);


    for(int i=0; i<grid_dim; i++) {
        al_draw_line(view.x + i * view.cell_l, view.y, view.x + i * view.cell_l, view.y + view.l,  RGBA(0,0,0,0.2), 1);
        al_draw_line(view.x, view.y + i * view.cell_l, view.x + view.l, view.y + i * view.cell_l,  RGBA(0,0,0,0.2), 1);
    }
    al_draw_rectangle(view.x, view.y, view.x + view.l, view.y + view.l,  RGBA(0,0,0,1), 1);
  

    car_t car;
    car.width = msg->car_w/512*SIZE;
    car.length = msg->car_l/512*SIZE;
    
    int xp = grid_dim/2, yp = grid_dim - (car.length/10*8)/view.cell_l;

    /*
    float_point_t o;
    o.x = view.x + view.cell_l/2 + view.l/2 - car.width/2;
    o.y = view.y + view.l - car.length;
    viz_rect(o, car.width, car.length, CAR_COLOR,1);
    */

    float_point_t p = grid2view(xp, yp, view);
    float ang = -M_PI/2;
    float_point_t p0 = p;
    for(int i=0; i<msg->steer_l; i++) {
        float steer_ang = ((float) msg->steer) /100.0 * M_PI/4; 

        p.x = p.x + cos(ang + steer_ang)*view.cell_l;
        p.y = p.y + sin(ang + steer_ang)*view.cell_l;
        ang = ang + (view.cell_l/car.length) * tan(steer_ang);
        //al_draw_line(p.x, p.y, p0.x, p0.y, CAR_COLOR, 2);
        p0 = p;
    }


    float_point_t mouse = viz_mouse();
    //al_draw_circle(mouse.x, mouse.y, 2, VIEW_COLOR, 1);
    al_flip_display();
}


int main(int argc, char **argv) {


    ros::init(argc, argv, "dinonav_viewer");

    ros::NodeHandle n;

    ros::Subscriber m_sub = n.subscribe("dinonav/stat", 1,   map_recv);
    ros::Subscriber l_sub = n.subscribe("scan", 1,   laser_recv);

    viz_init(SIZE+20, SIZE+20 + 200);
    while(ros::ok() && viz_update()) {
        ros::spinOnce();
    }

    viz_destroy();
    return 0;
}
