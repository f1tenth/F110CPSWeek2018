# Simple Obstacle Avoidance

## Demonstration

1. New ROS users:

Execute through the launch file by bringing up the terminal
```
roslaunch basic_obstacle_avoidance basic_obstacle_avoidance.launch
```

2. Advanced ROS users:

Individually start the nodes in separate terminals:
```
roscore
rosrun urg_node urg_node _ip_address:=192.168.1.X
rosrun basic_obstacle_avoidance steering_control.py
rosrun basic_obstacle_avoidance throttle_control.py
rosrun racecar_control basic_racecar_control.py
```
