set(execName visual_odometry)
cmake_minimum_required(VERSION 2.8.3)
project(${execName})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)
find_package(ZED 2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CUDA REQUIRED)

if(NOT WIN32)
     SET(SPECIAL_OS_LIBS "pthread" "X11")
    add_definitions(-Wno-write-strings)
endif()

catkin_package(
 INCLUDE_DIRS include
 #LIBRARIES
 CATKIN_DEPENDS nav_msgs roscpp  std_msgs tf2_ros tf2_geometry_msgs
 #DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

ADD_EXECUTABLE(${execName} src/visual_odometry_node.cpp)
add_definitions(-std=c++11 -g -O3)

target_link_libraries(${execName}
    ${catkin_LIBRARIES}
    ${ZED_LIBRARIES}
    ${CUDA_LIBRARIES} ${CUDA_NPP_LIBRARIES_ZED}
    ${OpenCV_LIBS}
)
