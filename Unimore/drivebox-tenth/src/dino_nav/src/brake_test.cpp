#include "ros/ros.h"
 
#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif

#include "race/drive_param.h"
#include "dino_nav/Stat.h"

#include "sensor_msgs/LaserScan.h"
#include "common.h"
#include "dinonav.h"
#include "perception.h"
#include "planning.h"
#include "actuation.h"


enum state_e { PREPARE, START, BRAKE, END };

float update_lidar_speed(float dst, ros::Time time);

const int MAX_TESTS = 256;
const int MAX_VELS = 1024;
struct test_t {
    float speed, brake;
    float vels[MAX_VELS];
    float zed_vels[MAX_VELS];
    int vels_n;
    float speed_reached, start, brake_start, brake_end;
} tests[MAX_TESTS];
int test_num;

extern ros::Publisher drive_pub, stat_pub;
extern dinonav_t nav;

const float BRK_DST = 14;
const int MIN_DST = 2;
float target_throttle = 0;

bool KILL = false;

void print_tests(float_point_t pos, int cur_test) {

    for(int i=0; i<cur_test; i++) {
        test_t *t = &tests[i];
        float offs = 40*i;

        viz_text(pos.x, pos.y + offs, 12, VIEW_COLOR, 
            "TEST %d, speed: %f  brake: %f", i, t->speed, t->brake);
        viz_text(pos.x, pos.y + offs +10, 12, VIEW_COLOR, 
            "dists: start %f, brake start: %f  brake end: %f", t->start, t->brake_start, t->brake_end);
         viz_text(pos.x, pos.y + offs +20, 12, VIEW_COLOR, 
            "max speed %f, acc dist %f, brake dist %f", 
            t->speed_reached, t->start - t->brake_start, t->brake_start - t->brake_end);           
    }
}

void print_test_text(int id) {
    test_t *t = &tests[id];

    printf("\n");
    printf("-------- TEST %d --------\n", id);
    printf("speed: %f  brake: %f\n", t->speed, t->brake);
    printf("dists: start %f, brake start: %f  brake end: %f\n", 
        t->start, t->brake_start, t->brake_end);
    printf("max speed %f, acc dist %f, brake dist %f\n", 
        t->speed_reached, t->start - t->brake_start, t->brake_start - t->brake_end); 
    printf("-------------------------\n");
    printf("vels: ");
    for(int i=0; i<t->vels_n; i++)
        printf("%f ", t->vels[i]);
    printf("\n");
    printf("-------------------------\n");
    printf("zed-vels: ");
    for(int i=0; i<t->vels_n; i++)
        printf("%f ", t->zed_vels[i]);
    printf("\n");
    printf("-------------------------\n\n");
}

float mean_ray(std::vector<float> v, int idx, int l) {
    float sum = 0;
    
    for(int i=idx-l; i<=idx+l; i++)
        sum += v[i];
    return sum/(l*2+1);
}

void run_test(float &throttle, float &steer, float wall_dist, view_t &view) {

    static float old_throttle = 0;
    static state_e state = PREPARE;
 
    float lidar_speed = nav.estimated_speed;
    if(wall_dist < 50)
        lidar_speed = update_lidar_speed(wall_dist, ros::Time::now());
    float zed_speed = nav.estimated_speed;
    //viz_text(view.x + view.l + 20, view.y +100, 18, VIEW_COLOR, "lidar speed: %f", lidar_speed);
    //viz_text(view.x + view.l + 20, view.y +120, 18, VIEW_COLOR, "zed speed:   %f", zed_speed);
    
    static int n =0;
    n++;
    if(n<100)
        return;
    if(n == 100)
        printf("test 0 PREPARE\n");
    if(n%100 == 0)
        printf("current dst = %f\n", wall_dist);


    static int current_test = 0;

    test_t *test = &tests[current_test];

    if( (state == START || state == BRAKE) && test->vels_n < MAX_VELS-1) {
        test->vels[test->vels_n] = lidar_speed;
        test->zed_vels[test->vels_n] = zed_speed;
        test->vels_n++;
    }

    switch(state) {

    case PREPARE:
        viz_text(view.x + view.l + 20, view.y +140, 15, VIEW_COLOR, "test %d status: PREPARE", current_test);
        test->start = wall_dist;
        printf("test throttle %f\n", target_throttle);
        state = START;
        old_throttle = 0;
    
        break;

    case START:
      viz_text(view.x + view.l + 20, view.y +140, 15, VIEW_COLOR, "test %d status: START", current_test);
        if(wall_dist < MIN_DST) {
	        state = BRAKE;
	        printf("emergency brake");
	    } 
        if(wall_dist < BRK_DST) {
            state = BRAKE;
            test->brake_start = wall_dist;
            test->speed_reached = lidar_speed;
            printf("test %d BRAKE\n", current_test);
        } else {
            if(old_throttle < target_throttle)
                old_throttle += 0.7;
            throttle = old_throttle;
        }
        break;

    case BRAKE:
      viz_text(view.x + view.l + 20, view.y +140, 15, VIEW_COLOR, "test %d status: BRAKE", current_test);
        if(lidar_speed > 0.01 || lidar_speed < -0.01) {
            throttle = -100;
        } else {
            test->brake_end = wall_dist;
            print_test_text(current_test);
            current_test++;
            if(current_test < test_num) {
                state = PREPARE;
                printf("test %d PREPARE\n", current_test);
            } else {
                state = END;
                printf("ALL TEST ENDED\n");
                KILL= true;
            }
        }
        break;
    
    case END:
        viz_text(view.x + view.l + 20, view.y +140, 15, VIEW_COLOR, "ALL TESTS EXECUTED");
        break;
    }

    float_point_t test_p;
    test_p.x = view.x + view.l + 20;
    test_p.y = view.y + 180;
    print_tests(test_p, current_test);
}

void laser_reciver(const sensor_msgs::LaserScan::ConstPtr& msg) {
     
    viz_clear();

    ros::WallTime time_debug = ros::WallTime::now(); //time record
    //ROS_INFO("Scan recived: [%f]", msg->scan_time);
    nav.conf.grid_dim = 800;
    nav.conf.zoom = 10.0;
    init(nav.view, nav.car, nav.grid);
    
    perception(nav, msg);

    draw_car(nav.conf, nav.view, nav.car);
    draw_grid(nav.grid, nav.view);   

    float wall_y = 0;   
    int idx = nav.grid.middle_id;

    float wall_dist = 1000;
    if(nav.grid.points[idx].x == nav.conf.grid_dim/2) {
        wall_y = nav.grid.points[idx].y;
    }
    
    float_point_t cp = grid2view(nav.car_pos.x, nav.car_pos.y, nav.view);
    float_point_t gp = grid2view(nav.car_pos.x, wall_y, nav.view);
    wall_dist = point_dst(cp, gp)*((nav.conf.zoom*2)/nav.view.l);

    viz_line(cp, gp, PATH_COLOR, 1);
    viz_text(cp.x + 5, (cp.y + gp.y)/2, 15, VIEW_COLOR, "%f", wall_dist);


    float throttle = 0;
    float steer = 0;
    run_test(throttle, steer, wall_dist, nav.view);
   
    race::drive_param drive_msg;
    drive_msg.angle = 2;
    drive_msg.velocity = throttle;
    drive_pub.publish(drive_msg);

    draw_drive_params(nav.view, drive_msg.velocity, drive_msg.angle, 
        nav.estimated_speed, nav.estimated_acc, nav.target_acc);
    viz_flip();

    //check if the computation was taken in more than 0.025 secs
    double time = (ros::WallTime::now() - time_debug).toSec();
    #ifndef TIME_PROFILER
    if(time >= 1/40.0f)
        printf("iter time exceded: %lf\n", time);
    #else
        double start = time_debug.toSec();
        double end = start + time;
        printf("DINONAV %lf %lf %lf\n", start, end, time);
    #endif

    //PUB stats for viewer
    dino_nav::Stat stat;
    stat.car_w = nav.car.width;
    stat.car_l = nav.car.length;

    stat.grid_size = nav.grid.size;
    std::vector<signed char> vgrd(nav.grid.data, nav.grid.data+(nav.grid.size*nav.grid.size));
    stat.grid = vgrd;
    stat.zoom = nav.conf.zoom;

    stat.steer_l = 0;
    stat.throttle = drive_msg.velocity;
    stat.steer = drive_msg.angle;
    stat.speed = nav.estimated_speed;
    stat.acc = nav.estimated_acc;
    geometry_msgs::Pose pose;
    stat.pose = pose;
    stat_pub.publish(stat);
}


void init_tests() {
    test_num = 1; 
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "dinonav");

    target_throttle = atoi(argv[1]);

    ros::NodeHandle n;

    ros::Subscriber ssub = n.subscribe("scan", 1, laser_reciver);
    ros::Subscriber osub = n.subscribe("zed/odom", 1, odom_recv);
    drive_pub = n.advertise<race::drive_param>("drive_parameters", 1);
    stat_pub = n.advertise<dino_nav::Stat>("dinonav/stat", 1);          //stats for viewer

    dynamic_reconfigure::Server<dino_nav::DinonavConfig> server;
    dynamic_reconfigure::Server<dino_nav::DinonavConfig>::CallbackType f;
    f = boost::bind(&reconf, _1, _2);
    server.setCallback(f);

    init_tests();

    viz_init(1000,700);
    while(!KILL && ros::ok() && viz_update()) {
        ros::spinOnce();
    }

    viz_destroy();
    return 0;
}

float update_lidar_speed(float dst, ros::Time time) {

    static bool init=false;
    const int VELS_DIM = 10;
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
    vels[now].pos.x = 0;
    vels[now].pos.y = dst;
    vels[now].t = time;

    float speed = 0;
    for(int i=1; i<VELS_DIM/2; i++) {
        int idx = (now+i) % VELS_DIM;

        double dx = vels[now].pos.x - vels[idx].pos.x;
        double dy = vels[now].pos.y - vels[idx].pos.y;
        double dst = sqrt(dx*dx + dy*dy);
        double dt = (vels[now].t - vels[idx].t).toSec();

        //sum for mean
        speed += dst/dt;
    }
    speed /= (VELS_DIM/2 -1);

    vels[now].vel = nav.estimated_speed;
    now = (now+1) % VELS_DIM;

    return speed;
}
