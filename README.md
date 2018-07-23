# F1/10 @ CTU
Workspace containing whatever is needed to run F1/10 car at CTU.

**Headstart**: This file is written in a way to be more generic. It means that distro versions may be omitted and only common designation is kept - (e.g. for package `ros-indigo-std-msgs` only `-std-msgs` is left)

## ROS over network
For communication over network, following environment variables needs to be
exported on remote:
```
export ROS_MASTER_URI=http://IP_ADDR_OF_TEGRA:11311
export ROS_IP=LOCAL_IP_ADDR
```

and the following on the Tegra:
```
export ROS_IP=LOCAL_IP_ADDR
```

## Installing / Getting started

### Compile the project
The main ROS package (rericha\_racing) can be compiled using the catkin tool

```
cd catkin_ws
catkin_make
source devel/setup.bash
```

### Message packages
Installing ROS full already includes almost all used messages. However some of them are missing:

### Teensy firmware
#### In schroot
- catkin workspace already sourced

```
sudo apt-get install ros-indigo-rosserial-arduino ros-indigo-rosserial
rm -rf ./hw/teensy-template/libraries/ros_lib/
rosrun rosserial_arduino make_libraries.py ./hw/teensy-template/libraries/
```
- install rosserial packages
- if present, delete ros libraries first
- create ros libraries for teensy

#### In host system
```
cd ./hw/teensy-template/
git am ../teensy-drive/teensy-template.patch
make
make upload
```

- change to teensy directory
- patch teensy template (only once, after submodule init)
- build the teensy firmware
- upload to teensy board

### Run the competition code
```
roslaunch rericha_racing run_follow-the-gap.launch
```

## Licensing
This project is published under GPLv3.

## File tree
    f1tenth                         # this repository
    ├── catkin_ws                   # catkin workspace
    │   └── src                         # sources of ROS packages
    │       └── rericha_racing              # external, for simulations
    ├── deb                         # Debian metapackages
    ├── doc                         # documentation
    ├── hw                          # hardware, low level, firmware
    └── README.md                   # this readme file

## Contact
web pages: http://industrialinformatics.cz
e-mail: rericha@rtime.felk.cvut.cz
