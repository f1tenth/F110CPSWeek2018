# F1tenth navigation #
HipertLab

### Compile ###

Compile instruction for ubuntu 14.04

- install ROS Indigo following the upstream guide:
  http://wiki.ros.org/indigo/Installation/Ubuntu </br>
  be sure to install "ros-indigo-desktop-full"
    
- install dependencies
```bash
sudo apt install ros-indigo-urg* libzbar-dev liballegro5-dev
```
- update your local "ros.h" file
```bash
sudo cp src/hack/ros.h /opt/ros/indigo/include/ros/ros.h 
```

- compile
```bash
catkin_make
```

optional useful compile options are:
```bash
CATKIN_BLACKLIST_PACKAGES="foo; bar"  #exclude from compiling packages 
CMAKE_BUILD_TYPE=Debug                #build with debug symbols
VIZ=OFF or VIZ=ON                     #disable or enable dinonav visualization
TIME_PROFILER=OFF or TIME_PROFILER=ON #disable or enable time profiler
```
options must be declared after the make command preceded by "-D", ex:
```bash
catkin_make -DCATKIN_BLACKLIST_PACKAGES="zed_wrapper"
```
