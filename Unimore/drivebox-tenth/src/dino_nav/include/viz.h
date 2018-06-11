#ifndef VIZ_H
#define VIZ_H

#include <allegro5/allegro.h>
#include "common.h"
#include "dinonav.h"
#include "grid.h"
#include "pathfind.h"

#define RGBA(r, g, b, a)  al_map_rgba_f(r*a,g*a,b*a, a)

const ALLEGRO_COLOR VIEW_COLOR      = RGBA(0, 1, 0, 1.0);
const ALLEGRO_COLOR VIEW_GRID_COLOR = RGBA(0, 1, 0, 0.1);
const ALLEGRO_COLOR WALL_COLOR      = RGBA(0, 1, 1, 0.5);
const ALLEGRO_COLOR INFLATED_COLOR  = RGBA(0, 1, 1, 0.2);
const ALLEGRO_COLOR GATE_COLOR      = RGBA(1, 1, 1, 0.2);
const ALLEGRO_COLOR PATH_COLOR      = RGBA(1, 0, 0, 1.0);
const ALLEGRO_COLOR PATH_GRID_COLOR = RGBA(1, 0, 1, 0.2);
const ALLEGRO_COLOR CAR_COLOR       = RGBA(0, 1, 1, 1.0);
const ALLEGRO_COLOR LPATH_COLOR     = RGBA(0, 1, 1, 0.05);

bool viz_init(float w, float h);
bool viz_update();
void viz_destroy();
void viz_flip();
void viz_clear();

void viz_rect(float_point_t o, float w, float h, ALLEGRO_COLOR col, float thick);
void viz_circle(float_point_t p, float r, ALLEGRO_COLOR col, float thick);
void viz_line(float_point_t a, float_point_t b, ALLEGRO_COLOR col, float thick);
void viz_arc(float cx, float cy, float r, float start_theta, float delta_theta, 
             ALLEGRO_COLOR col, float thick);
void viz_triangle(float_point_t a, float_point_t b, float_point_t c, 
                  ALLEGRO_COLOR col, float thick);
             
void viz_text(float x, float y, int dim, ALLEGRO_COLOR col, const char *format, ...);

float_point_t viz_mouse();

void draw_drive_params(view_t &view, float throttle, float steer, float speed, float acc, float t_acc);
void draw_grid(grid_t &grid, view_t &view);
void draw_track(track_t &track, view_t &view);
void draw_orient(float yaw, float pitch, float roll, view_t &view);
void draw_car(conf_t &conf, view_t &view, car_t &car);
void draw_path(path_t &path);
void plot_floats(float *values, int start, int size, float min, float max,
                 float_point_t &pos, float_point_t &dim, const char *text);
#endif //VIZ_H

