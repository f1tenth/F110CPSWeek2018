#include <iostream>
#include <math.h>

#include "actuation.h"

#include "dinonav.h"
#include "pathfind.h"

#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif


void actuation(dinonav_t &nav, ackermann_msgs::AckermannDrive &drive_msg) {

    point_t part = nav.car_pos;
    point_t goal = nav.goal_pos;

    float_point_t start = grid2view(part.x, part.y, nav.view);
    float_point_t enter = nav.curve.b;
    float_point_t exit = grid2view(goal.x, goal.y, nav.view);

    if(nav.curve.b.x >0) {
        viz_circle(enter, 4, CAR_COLOR, 1);
        viz_circle(exit, 4, CAR_COLOR, 1);
        viz_line(start, enter, CAR_COLOR, 1);
        viz_line(enter, exit, CAR_COLOR, 1);

        viz_text((start.x + enter.x)/2, (start.y + enter.y)/2, 10, RGBA(1,0,1,1), "  %f", nav.curve_dst);
    } else {
        viz_circle(exit, 4, CAR_COLOR, 1);
    }

    //find front wall distence
    float front_wall_dist = point_dst(nav.curve.a, start);
    if(nav.curve.a.y <= 0) {
        front_wall_dist = 1000;
    } else { 
        front_wall_dist = view2meters(nav, front_wall_dist);
        float_point_t front_p = nav.curve.a;
        viz_line(start, front_p, CAR_COLOR, 1);
        viz_text(front_p.x, front_p.y, 10, VIEW_COLOR, "  %f", front_wall_dist);
    }

    nav.steer = calc_steer(start, nav.view, nav.car, nav.grid, nav.path, nav.steer_l);

    nav.throttle = calc_throttle(nav.conf, nav.view, nav.car, nav.track, nav.curve, 
        front_wall_dist, nav.estimated_speed, nav.estimated_acc, nav.target_acc);
    
    if(nav.throttle > nav.conf.throttle)
        nav.throttle = nav.conf.throttle;
    
    if(nav.steer < 0)
        nav.steer = nav.steer*nav.conf.oversteer_left; 
    if(nav.steer > 0)
        nav.steer = nav.steer*nav.conf.oversteer_right;

    drive_msg.speed = float(nav.throttle)/100;
    drive_msg.steering_angle = float(nav.steer)/100;
    
}


float calc_steer(float_point_t &start, view_t &view, car_t &car, grid_t &grid, path_t &path, int &steer_l) {
    float steer = 0;
    steer_l = 0;

    for(int i=0; i<path.size; i++) {
        point_t p = view2grid(path.data[i].x, path.data[i].y, view);
        setgrid(grid, p.x, p.y, PATH);
    }

    int best_steer = 0;
    int best_match = 0;
    for(int j=-100; j<100; j+=2) {
        float_point_t p = start;
        float ang = -M_PI/2;

        int match = 0;

        float_point_t p0 = p;
        for(int i=0; i<20; i++) {
            float steer_ang = ((float) j) /100.0 * M_PI/4; 

            p.x = p.x + cos(ang + steer_ang)*view.cell_l;
            p.y = p.y + sin(ang + steer_ang)*view.cell_l;
            ang = ang + (view.cell_l/car.length) * tan(steer_ang);
            
            viz_line(p, p0, LPATH_COLOR, 1);
            point_t gp = view2grid(p.x, p.y, view);
            int val = getgrid(grid, gp.x, gp.y);
            if(val == PATH) {
                viz_circle(p, 5, LPATH_COLOR, 1);
                match++;
                if(match > best_match) {
                    best_steer = j;
                    best_match = match;
                }
            } else if(val != EMPTY) {
                break;
            }
            p0 = p;
        }
    }

    float_point_t p = start;
    float ang = -M_PI/2;

    float_point_t p0 = p;
    for(int i=0; i<20; i++) {
        float steer_ang = ((float) best_steer) /100.0 * M_PI/4; 

        p.x = p.x + cos(ang + steer_ang)*view.cell_l;
        p.y = p.y + sin(ang + steer_ang)*view.cell_l;
        ang = ang + (view.cell_l/car.length) * tan(steer_ang);
        viz_line(p, p0, VIEW_COLOR, 2);
        p0 = p;
    }

    steer = best_steer;
    steer_l = 20;

    return steer;
}


float calc_throttle(conf_t &conf, view_t &view, car_t &car, track_t &track, segment_t &curve, 
    float front_dst, float estimated_speed, float estimated_acc, float &target_acc) {

    if(front_dst < 0.2)
       return -100;

    static float throttle = 0;
    float curve_safety = front_dst/conf.curve_safety;
        
    float delta = fabs(estimated_speed - curve_safety);
    if(estimated_speed > curve_safety) {
        if(throttle > 0)
            throttle = 0;
        throttle = -delta*conf.car_decel*10;
    } else {
        if(throttle < 0)
            throttle = 10;
        throttle += delta*2;
    } 

    if(throttle != throttle)
        throttle = 0;
    throttle = fclamp(throttle, -100, 100);

/*
    CURVE PASSED

    static int in_curve = 0;
    float_point_t pos;
    pos.x = view.x + view.l/2;
    pos.y = view.y + view.l - conf.ahead_offset - car.length*1;
    
    if(curve.a.x > 0) {
        float Cdst = point_dst(curve.a, pos);
        float Clen = car.width*track.sects[track.cur_sect].enter;

        if(Cdst < Clen)
            in_curve++;
        
        if(in_curve > 5 && Cdst > Clen) {
            printf("curve passed\n");
            track.cur_sect = (track.cur_sect +1) % track.sects_n;
            in_curve =0;
        }
    }
*/
    return throttle;
}
