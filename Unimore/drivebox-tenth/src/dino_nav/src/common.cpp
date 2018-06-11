#include <math.h> 
#include "common.h"

/**
    Compute angle beetwen 2 points in term of car steering (-100; +100)
*/
float points_angle(float x_part, float y_part, float x_goal, float y_goal) {
 
    float ang = atan2(y_goal - y_part, x_goal - x_part)*180/M_PI +90;
    if(ang > 180) ang -= 360;
    // ang : 45 = new_ang : 100
    ang = ang*100/45;
 
    return ang;
}

/**
    Compute angle beetwen 2 points in rad
*/
float points_angle_rad(float x_part, float y_part, float x_goal, float y_goal) {

    float ang = atan2(y_goal - y_part, x_goal - x_part);
    return ang;
}

/**
    Rotate point p around origin of theta angle (rad)
*/
void rotate_point(float_point_t &p, float_point_t &o, float theta) {
    int x = o.x + (p.x - o.x)*cos(theta) - (p.y - o.y)*sin(theta);
    int y = o.y + (p.y - o.y)*cos(theta) + (p.x - o.x)*sin(theta);

    p.x = x; p.y = y;
}

/**
    Get max value of an array of float
*/
float get_max_value(const float *a, int dim) {
    float max = a[0];
    for(int i=1; i<dim; i++)
        if(a[i] > max) max = a[i];
    return max;
}

/**
    Compute points distance 
*/
float point_dst(float_point_t &a, float_point_t &b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

/*
    Tell if a point is in front or back of a segment
    r>0 = FRONT, r<0 = BACK, r == ALINE 
*/
float point_is_front(segment_t &s, float_point_t &p){
 
    float r = ((s.b.x - s.a.x)*(p.y - s.a.y) - (s.b.y - s.a.y)*(p.x - s.a.x));
    return r;
}
 
/**
    Clamp a float value beetwen min and max
*/
float fclamp(float val, float min, float max) {
    if(val > max) return max;
    else if(val < min) return min;
    else return val;
}

/** 
    Find the points where the two circles intersect. 
*/
int find_circle_intersections(float cx0, float cy0, float radius0, float cx1, float cy1, float radius1, 
    float_point_t &intersection1, float_point_t &intersection2) {

    // Find the distance between the centers. 
    float dx = cx0 - cx1; 
    float dy = cy0 - cy1; 
    double dist = sqrt(dx * dx + dy * dy); 
 
    // See how many solutions there are. 
    if (dist > radius0 + radius1) 
    { 
        // No solutions, the circles are too far apart. 
        intersection1.x = intersection1.y = -1; 
        intersection2.x = intersection2.y = -1; 
        return 0; 
    } 
    else if (dist < fabs(radius0 - radius1)) 
    { 
        // No solutions, one circle contains the other. 
        intersection1.x = intersection1.y = -1; 
        intersection2.x = intersection2.y = -1; 
        return 0; 
    } 
    else if ((dist == 0) && (radius0 == radius1)) 
    { 
        // No solutions, the circles coincide. 
        intersection1.x = intersection1.y = -1; 
        intersection2.x = intersection2.y = -1; 
        return 0; 
    } 
    else 
    { 
        // Find a and h. 
        double a = (radius0 * radius0 - 
            radius1 * radius1 + dist * dist) / (2 * dist); 
        double h = sqrt(radius0 * radius0 - a * a); 
 
        // Find P2. 
        double cx2 = cx0 + a * (cx1 - cx0) / dist; 
        double cy2 = cy0 + a * (cy1 - cy0) / dist; 
 
        // Get the points P3. 
        intersection1.x = (float)(cx2 + h * (cy1 - cy0) / dist); 
        intersection1.y = (float)(cy2 - h * (cx1 - cx0) / dist); 
 
        intersection2.x = (float)(cx2 - h * (cy1 - cy0) / dist); 
        intersection2.y = (float)(cy2 + h * (cx1 - cx0) / dist); 
 
        // See if we have 1 or 2 solutions. 
        if (dist == radius0 + radius1) return 1; 
        return 2; 
    } 
} 
 
/**
    find the tangents points of a circle
*/
bool find_circle_tang(float_point_t center, float radius, 
    float_point_t external_point, float_point_t &pt1, float_point_t &pt2) {
         
    // Find the distance squared from the 
    // external point to the circle's center. 
    double dx = center.x - external_point.x; 
    double dy = center.y - external_point.y; 
    double D_squared = dx * dx + dy * dy; 
    if (D_squared < radius * radius) 
    { 
        pt1.x = pt1.y = pt2.x = pt2.y = -1; 
        return false; 
    } 
 
    // Find the distance from the external point 
    // to the tangent points. 
    double L = sqrt(D_squared - radius * radius); 
 
    // Find the points of intersection between 
    // the original circle and the circle with 
    // center external_point and radius dist. 
    find_circle_intersections( 
        center.x, center.y, radius, 
        external_point.x, external_point.y, (float)L, 
        pt1, pt2); 
 
    return true; 
} 