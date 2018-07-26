# Getting started with AutoCar

## Table of Contents

1. [Requirements](#Requirements)
2. [Installation](#Installation)
3. [Execution](#Execution)

## Requirements

- ROS Indigo
    - rosserial_python
- Ubuntu 14.04
- F1Tenth Platform
- Jetson TK1
- Hokuyo LiDAR UST-10LX

## Installation

Install ROS on the Jetson and on your development machine following the tutorials available at [F1Tenth](http://f1tenth.org)

## Execution

### What nodes to run

Make sure your terminals are sourced.

1. `roscore`
2. Communications with vehicle

    `rosrun rosserial_python serial_node.py /dev/ttyACM0`
3. Emergency stop node

    `rosrun emergency_stop estop_filter.py`
4. In one terminal, run the following:

    `./setup_bridge.py`

    `rosrun urg_node urg_node _ip_address:=192.168.1.11`
5. Controller

    `rosrun pure_pursuit manhattan_control.py`

## Network config

- Jetson's ip address is 192.168.1.1

- Lidar's ip address is 192.168.1.11

- Remote computer's ip address is 192.168.1.112
