cmake_minimum_required(VERSION 2.8.3)
project(rericha_racing)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    geometry_msgs
    tf
    roslib
)

add_message_files(
    FILES
    drive_values.msg
    pwm_high.msg
    CarControlData.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(follow_the_gap src/follow_the_gap/follow_the_gap.cpp src/follow_the_gap/obstacle.cpp src/follow_the_gap/gap.cpp src/follow_the_gap/corner.cpp)

target_link_libraries(follow_the_gap
  ${catkin_LIBRARIES}
)
