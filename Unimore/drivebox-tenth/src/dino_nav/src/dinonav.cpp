#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "grid.h"
#include "pathfind.h"
#include "common.h"

#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif

#include "dinonav.h"
#include "perception.h"
#include "planning.h"
#include "actuation.h"

#include "dino_nav/Stat.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "std_msgs/Float32.h"

ros::Publisher drive_pub, stat_pub;

dinonav_t nav;  //contains all computed info
geometry_msgs::Pose pose;

/**
    Reconf callback, view "Dinonav.cfg" for specifications.
    Reconf can be maneged with "rqt_reconfigure" package
*/
void reconf(dino_nav::DinonavConfig &config, uint32_t level) {
    printf("\n");
    printf("################ RECONFIGURE ################\n");
    printf("throttle\t\t%d\ninflation\t\t%d\n", config.throttle, config.inflation);
    printf("grid_dim\t\t%d\nzoom\t\t\t%lf\n", config.grid_dim, config.zoom);
    printf("ahead_offset\t\t%d\nenable\t\t\t%d\n", config.ahead_offset, config.enable);
    printf("curve_safety\t\t%lf\ncar_decel\t\t%lf\n", config.curve_safety, config.car_decel);
    printf("#############################################\n\n");

    nav.conf.throttle     = config.throttle;
    nav.conf.inflation    = config.inflation;
    nav.conf.grid_dim     = config.grid_dim;
    nav.conf.zoom         = config.zoom;
    nav.conf.ahead_offset = config.ahead_offset;
    nav.conf.enable       = config.enable;
    nav.conf.curve_safety = config.curve_safety;
    nav.conf.car_decel    = config.car_decel;

    nav.conf.dist_from_center = config.dist_from_center;
    nav.conf.oversteer_left   = config.oversteer_left;
    nav.conf.oversteer_right  = config.oversteer_right;
    nav.conf.lidar_pos        = config.lidar_pos;
}

void init_view(view_t &view, int size) {
    view.x = 10; view.y = 10;
    view.l = 512;
    view.cell_l = view.l / float(size);
}

void init_car(car_t &car, view_t &view, float zoom) {
    float mul = (view.l/100); //zoom factor

    //empiric car dimensions
    car.length = (18.0f/zoom)*mul;     
    car.width  = (10.0f/zoom)*mul;
}

void init(view_t &view, car_t &car, grid_t &grid) {

    init_view(view, nav.conf.grid_dim);
    init_car(car, view, nav.conf.zoom);

    //init grid memory first time
    //TODO: place on main
    static int *grid_addr=NULL;
    if(grid_addr == NULL)
        grid_addr = new int[GRID_MAX_DIM*GRID_MAX_DIM];
    grid.data = grid_addr;
    init_grid(grid, nav.conf.grid_dim);
}

/**
    laserscan callback, executed at every lidar scan recived.
    All computation is there.
*/
void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg) {
    viz_clear();

    init(nav.view, nav.car, nav.grid);
    perception(nav, msg);
    
    draw_car(nav.conf, nav.view, nav.car);
    draw_grid(nav.grid, nav.view);   

    planning(nav);
    
    draw_orient(nav.yaw, nav.pitch, nav.roll, nav.view);
    //draw_track(nav.track, nav.view);
    draw_path(nav.path);

    ackermann_msgs::AckermannDrive drive_msg;
    actuation(nav, drive_msg);
    
    //limit throttle
    nav.throttle = fclamp(nav.throttle, -100, nav.conf.throttle); 
    if(nav.conf.enable) 
        drive_pub.publish(drive_msg);
    draw_drive_params(nav.view, nav.throttle, nav.steer, nav.estimated_speed, nav.estimated_acc, 0);    

    float_point_t pos = { nav.view.x + nav.view.l +10, 10 };
    float_point_t dim = { 250, 250 };
    plot_floats(nav.speeds, nav.speeds_idx, 256, 0, 10, pos, dim, "speed");
    pos.y += 300;
    plot_floats(nav.imu_acc, nav.imu_acc_idx, 256, -40, 40, pos, dim, "acceleration");

    //PUB stats for viewer
    dino_nav::Stat stat;
    stat.scan.angle_min = msg->angle_min;      
    stat.scan.angle_max = msg->angle_max;
    stat.scan.angle_increment = msg->angle_increment;
    stat.scan.ranges = msg->ranges;

    stat.car_w = nav.car.width;
    stat.car_l = nav.car.length;

    stat.grid_size = nav.grid.size;
    std::vector<signed char> vgrd(nav.grid.data, nav.grid.data+(nav.grid.size*nav.grid.size));
    stat.grid = vgrd;
    stat.zoom = nav.conf.zoom;

    stat.steer_l = nav.steer_l;
    stat.throttle = drive_msg.speed;
    stat.steer = drive_msg.steering_angle;
    stat.speed = nav.estimated_speed;
    stat.acc = nav.estimated_acc;
    stat.pose = pose;
    stat_pub.publish(stat);

    viz_flip();
}

/**
    Update nav.estimated_speed, based on odometry position change.
    It uses a rolling array of lasts positions and time and it make
    a mean of computed speeds for every old values to now. 
*/
void update_speed(geometry_msgs::Point p, ros::Time time) {

    static bool init=false;
    const int VELS_DIM = 4;
    static vels_t vels[VELS_DIM];
    static int now = 0;

    if(!init) {
        for(int i=0; i< VELS_DIM; i++) {
            vels[i].t = time;
            vels[i].pos.x = 0;
            vels[i].pos.y = 0; 
	    vels[i].vel = 0;
        }
        init = true;
    }
    vels[now].pos.x = p.x;
    vels[now].pos.y = p.y;
    vels[now].t = time;

    nav.estimated_speed = 0;
    for(int i=1; i<VELS_DIM/2; i++) {
        int idx = (now+i) % VELS_DIM;

        double dx = vels[now].pos.x - vels[idx].pos.x;
        double dy = vels[now].pos.y - vels[idx].pos.y;
        double dst = sqrt(dx*dx + dy*dy);
        double dt = (vels[now].t - vels[idx].t).toSec();

        //sum for mean
        nav.estimated_speed += dst/dt;
    }
    nav.estimated_speed /= (VELS_DIM/2 -1);
    nav.estimated_acc = (nav.estimated_speed - vels[(now+1)%VELS_DIM].vel) / 
      (vels[now].t - vels[(now+1)%VELS_DIM].t).toSec(); 

    vels[now].vel = nav.estimated_speed;
    now = (now+1) % VELS_DIM;


    nav.speeds[nav.speeds_idx % 256] = nav.estimated_speed;
    nav.speeds_idx++;
}

/**
    PoseStamped callback
*/
void pose_recv(const geometry_msgs::Pose2D::ConstPtr& msg) {
    geometry_msgs::Point p;
    p.x = msg->x;
    p.y = msg->y;

    pose.position.x = p.x;
    pose.position.y = p.y;
    update_speed(p, ros::Time::now());
}

/**
    Odometry callback
*/
void odom_recv(const nav_msgs::Odometry::ConstPtr& msg) {

    pose = msg->pose.pose;
    //update_speed(msg->pose.pose.position, msg->header.stamp);
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    nav.estimated_speed = sqrt(vx*vx + vy*vy);

    //update nav.yaw value
    tf::Quaternion q(   pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, nav.yaw);
}

/** 
    Track zone reset callback
*/
void track_zone_recv(const std_msgs::Int32::ConstPtr &msg) {

    nav.track.cur_sect = msg->data;
    printf("track zone reset to %d\n", msg->data);
}

void imu_recv(const sensor_msgs::Imu::ConstPtr& msg) {
    const int MEAN_LENGHT = 2;

    float sum = 0;
    for(int i=nav.imu_acc_idx - MEAN_LENGHT; i<nav.imu_acc_idx; i++) {
        int idx = i % 256;

        sum += nav.imu_acc[idx];
    }

    nav.imu_acc[nav.imu_acc_idx % 256] = (sum + msg->linear_acceleration.y)/(MEAN_LENGHT+1);
    nav.imu_acc_idx++;

    //update nav.yaw value
    tf::Quaternion q(   msg->orientation.x, msg->orientation.y,
                        msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3(q).getRPY(nav.roll, nav.yaw, nav.pitch);
}


