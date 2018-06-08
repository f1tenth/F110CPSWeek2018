#ifndef COMMON_H
#define COMMON_H

struct point_t {
    int x, y;
};

struct float_point_t {
    float x, y;
};

struct segment_t {
    float_point_t a, b;
    int dir;
};

float points_angle(float x_part, float y_part, float x_goal, float y_goal);
float points_angle_rad(float x_part, float y_part, float x_goal, float y_goal);

void rotate_point(float_point_t &p, float_point_t &o, float theta);
float get_max_value(const float *a, int dim);
float point_dst(float_point_t &a, float_point_t &b);

float point_is_front(segment_t &s, float_point_t &p);
float fclamp(float val, float min, float max);

bool find_circle_tang(float_point_t center, float radius, 
    float_point_t external_point, float_point_t &pt1, float_point_t &pt2);
    
#endif //COMMON_H