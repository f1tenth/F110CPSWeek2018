#ifndef DUMMYVIZ_H
#define DUMMYVIZ_H

#define RGBA(r, g, b, a)  0

#define VIEW_COLOR      0
#define VIEW_GRID_COLOR 0
#define WALL_COLOR      0
#define INFLATED_COLOR  0
#define GATE_COLOR      0
#define PATH_COLOR      0
#define PATH_GRID_COLOR 0
#define CAR_COLOR       0
#define LPATH_COLOR     0

#define viz_init(w, h)  true
#define viz_update()    true
#define viz_destroy()   ;
#define viz_flip()      ;
#define viz_clear()     ;

#define viz_rect(o, w, h, col, thick)   ;
#define viz_circle(p, r, col, thick)    ;
#define viz_line(a, b, col, thick)      ;
#define viz_arc(cx, cy, r, start_theta, delta_theta, col, thick)    ;
#define viz_triangle(a, b, c, col, thick)                           ;
             
#define viz_text(x, y, dim, col, format, ...)   ;

#define viz_mouse() 0

#define draw_drive_params(view, throttle, steer, speed, acc, t_acc)     ;
#define draw_grid(grid, view)                                           ;
#define draw_track(track, view)                                         ;
#define draw_orient(yaw, p, r, view)                                    ;
#define draw_car(conf, view, car)                                       ;
#define draw_path(path)                                                 ;
#define plot_floats(values, start, size, min, max, pos, dim, text)      ;

#endif //DUMMYVIZ_H

