#include <iostream>
#include <math.h>

#include "planning.h"
#include "pathfind.h"

#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif

int path_line(grid_t &grid, int x1, int y1, int x2, int y2, path_t &path) {
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
        if(getgrid(grid, xx, yy) == -1)
            break;
        path.data[path.size].x = xx;
        path.data[path.size].y = yy;
        path.size++;
    }

    return steps;
}


void planning(dinonav_t &nav) {
 
    nav.goal_pos.x = -1;
    nav.goal_pos.y = -1;

    int gate_idx = choosegate(nav.grid, nav.car_pos.x, nav.car_pos.y);
    nav.curve = calc_curve(nav.grid, gate_idx, grid2view(nav.car_pos.x, nav.car_pos.y, nav.view), 
                            nav.view, nav.car, nav.track, nav.conf);

    nav.curve_dst = -1;
    if(nav.curve.a.x > 0) {
        float_point_t start = grid2view(nav.car_pos.x, nav.car_pos.y, nav.view);

        nav.curve_dst = fabs(start.y - nav.curve.a.y);
        nav.curve_dst = view2meters(nav, nav.curve_dst);
    
        if(point_dst(start, nav.curve.a) < point_dst(nav.curve.a, nav.curve.b)) {
            nav.curve.a.x = -1; nav.curve.a.y = -1;
            nav.curve.b.x = -1; nav.curve.b.y = -1;
        }
    }

    nav.goal_pos.x = (nav.grid.gates[gate_idx].s.x + nav.grid.gates[gate_idx].e.x)/2;
    nav.goal_pos.y = (nav.grid.gates[gate_idx].s.y + nav.grid.gates[gate_idx].e.y)/2;

    setgrid(nav.grid, nav.goal_pos.x, nav.goal_pos.y, 0);

    nav.path = pathfinding(nav.grid, nav.view, nav.car, nav.car_pos, nav.goal_pos, nav.curve);
    /*
    nav.path.start = 0;
    nav.path.size = 0;
    if(nav.curve.a.x < 0) {
        path_line(nav.grid, nav.car_pos.x, nav.car_pos.y, nav.goal_pos.x, nav.goal_pos.y, nav.path);
    } else {
        point_t enter = view2grid(nav.curve.b.x, nav.curve.b.y, nav.view);
        path_line(nav.grid, nav.car_pos.x, nav.car_pos.y, enter.x, enter.y, nav.path);
        path_line(nav.grid, enter.x, enter.y, nav.goal_pos.x, nav.goal_pos.y, nav.path);
    } 

    for(int i=0; i<nav.path.size; i++) {
        nav.path.data[i] = grid2view(nav.path.data[i].x, nav.path.data[i].y, nav.view);
    }*/
}


void draw_signal(float_point_t center, float r, dir_e d) {

    viz_circle(center, r, RGBA(1,1,1,1), 0);
    viz_circle(center, r, RGBA(1,0,0,1), r/5);
    float_point_t i, e;
    i.x = center.x - r/1.5;
    e.x = center.x + r/1.5;
    i.y = e.y = center.y; 
    viz_line(i, e, RGBA(0,0,0,1), r/10);
    
    float_point_t a, b;
    if(d == LEFT) {
        a.x = i.x + r/3;
        b.x = i.x + r/3;
    } else {
        i.x = e.x;
        a.x = i.x - r/3;
        b.x = i.x - r/3;
    }
    
    a.y = i.y - r/3;
    b.y = i.y + r/3;
    viz_triangle(i, a, b, RGBA(0,0,0,1), 0);
}


int choosegate(grid_t &grid, int px, int py) {

    int max_dim = 0;
    int idx = 0;
    for(int i=0; i<grid.gates_n; i++) {
        if(grid.gates[i].dim > max_dim)  {
            max_dim = grid.gates[i].dim;
            idx = i; 
        }
    }
    
    return idx;
}

segment_t calc_curve(grid_t &grid, int gate_idx, float_point_t start,
    view_t &view, car_t &car, track_t &track, conf_t &conf) {
    
    segment_t curve;
    curve.a.x = -1; curve.a.y = -1;
    curve.b.x = -1; curve.b.y = -1;

    point_t g1 = grid.gates[gate_idx].s;
    point_t g2 = grid.gates[gate_idx].e;

    point_t internal, external;
    int sign;

    int point1=-1, point2=-1;
    for(int i=0; i<grid.points_n; i++) {
        if( (grid.points[i].x == g1.x && grid.points[i].y == g1.y)     ||
            (grid.points[i].x == g2.x && grid.points[i].y == g2.y)    ) {
            
            if(point1 <0)
                point1 = i;
            else if(point2 <0) {
                point2 = i;
                break;
            }
        }
    }

    internal = grid.points[point1];
    external = grid.points[point2];
    float_point_t i = grid2view(internal.x, internal.y, view);
    float_point_t e = grid2view(external.x, external.y, view);
    float gate_ang = points_angle_rad(e.x, e.y, i.x, i.y);
    viz_text((i.x + e.x)/2, (i.y + e.y)/2, 15, RGBA(1,1,1,1), "%f", gate_ang);

    int point_idx;
    if(fabs(gate_ang) < 0.5f)
        return curve;
    if(gate_ang > 0) {
        point_idx = point1;
        sign = -1;
    } else if(gate_ang < 0) {
        point_idx = point2;
        sign = +1;
        internal = grid.points[point2];
        external = grid.points[point1];
    } 

    viz_line(   grid2view(internal.x, internal.y, view), 
                grid2view(external.x, external.y, view), VIEW_COLOR, 1);

    //calc curve intern
    float s_ang = 0;
    for(int i=0; i<6; i++) {
        int id = point_idx + i*sign;
        if(id <0 || id > grid.points_n-1)
            break;
        point_t s0 = grid.points[id];
        float_point_t a  = grid2view(internal.x, internal.y, view);
        float_point_t b = grid2view(s0.x, s0.y, view);
        float ang = points_angle_rad(a.x, a.y, b.x, b.y) - M_PI/2*sign;
        if(i == 0)
            s_ang = ang;
        else
            s_ang =  (s_ang + ang)/2;
    } 

    //reach opposite wall
    float_point_t int_v = grid2view(internal.x, internal.y, view);
    float_point_t opp_v, l_v;
    float width = 0;
    for(int i=1*conf.inflation +2; i<grid.size; i++) {
        opp_v.x = int_v.x + cos(s_ang)*view.cell_l*i;   opp_v.y = int_v.y + sin(s_ang)*view.cell_l*i;    
        point_t opp = view2grid(opp_v.x, opp_v.y, view);
        width = i;
        if(getgrid(grid, opp.x, opp.y) > GATE)
            break;
    }
    viz_line(int_v, opp_v, PATH_COLOR, 1);

    float curve_enter = point_dst(int_v, opp_v)*conf.dist_from_center;
    curve.a.x = int_v.x;
    curve.a.y = int_v.y;   
    curve.b.x = int_v.x + cos(s_ang)*curve_enter;   
    curve.b.y = int_v.y + sin(s_ang)*curve_enter;

    curve.dir = sign;

    //reach end wall
    s_ang -= M_PI/2*sign;
    for(int i=1*conf.inflation +2; i<grid.size; i++) {
        l_v.x = int_v.x + cos(s_ang)*view.cell_l*i;   l_v.y = int_v.y + sin(s_ang)*view.cell_l*i;    
        point_t l = view2grid(l_v.x, l_v.y, view);
        if(getgrid(grid, l.x, l.y) > GATE)
            break;
    }
    viz_line(int_v, l_v, PATH_COLOR, 1);

    return curve;
}
