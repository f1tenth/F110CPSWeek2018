cmake_minimum_required(VERSION 2.8.3)
project(dino_nav)

find_package(catkin REQUIRED COMPONENTS roslib roscpp rospy 
              std_msgs nav_msgs geometry_msgs genmsg race sensor_msgs
              dynamic_reconfigure message_generation)

find_package(PkgConfig)
pkg_search_module(ALLEGRO5 REQUIRED allegro-5.0)
pkg_search_module(ALLEGRO_PRIMITIVES REQUIRED allegro_primitives-5.0)
pkg_search_module(ALLEGRO_TTF REQUIRED allegro_ttf-5.0)

pkg_search_module(TinyXML tinyxml)

add_message_files(
   FILES
   FloatPoint.msg
   Point.msg
   Stat.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


generate_dynamic_reconfigure_options(
  cfg/Dinonav.cfg
)

catkin_package(
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS include
    DEPENDS TinyXML
)


include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

add_definitions("-std=c++11")


add_executable(dinonav src/main.cpp src/viz.cpp src/dinonav.cpp 
  src/perception.cpp src/planning.cpp src/actuation.cpp src/pathfind.cpp src/common.cpp src/grid.cpp)
add_dependencies(dinonav ${PROJECT_NAME}_gencfg)
add_dependencies(dinonav ${PROJECT_NAME}_generate_messages)
add_dependencies(dinonav race_generate_messages)
target_link_libraries(dinonav ${catkin_LIBRARIES} ${ALLEGRO5_LIBRARIES} ${ALLEGRO_PRIMITIVES_LIBRARIES}
                              ${ALLEGRO_TTF_LIBRARIES} ${TinyXML_LIBRARIES})

add_executable(brake_test src/brake_test.cpp src/viz.cpp src/common.cpp  src/dinonav.cpp
  src/perception.cpp src/planning.cpp src/actuation.cpp src/pathfind.cpp src/common.cpp src/grid.cpp)
add_dependencies(brake_test ${PROJECT_NAME}_gencfg)
add_dependencies(brake_test ${PROJECT_NAME}_generate_messages)
add_dependencies(brake_test race_generate_messages)
target_link_libraries(brake_test  ${catkin_LIBRARIES} ${ALLEGRO5_LIBRARIES} ${ALLEGRO_PRIMITIVES_LIBRARIES}
                                  ${ALLEGRO_TTF_LIBRARIES})

add_executable(viewer src/viewer.cpp src/common.cpp src/grid.cpp src/viz.cpp)
add_dependencies(viewer ${PROJECT_NAME}_gencfg)
add_dependencies(viewer ${PROJECT_NAME}_generate_messages)
add_dependencies(viewer race_generate_messages)
target_link_libraries(viewer  ${catkin_LIBRARIES} ${ALLEGRO5_LIBRARIES} ${ALLEGRO_PRIMITIVES_LIBRARIES}
                              ${ALLEGRO_TTF_LIBRARIES})

add_executable(viewerTesi src/viewerTesi.cpp src/common.cpp src/grid.cpp src/viz.cpp)
add_dependencies(viewerTesi ${PROJECT_NAME}_gencfg)
add_dependencies(viewerTesi ${PROJECT_NAME}_generate_messages)
add_dependencies(viewerTesi race_generate_messages)
target_link_libraries(viewerTesi  ${catkin_LIBRARIES} ${ALLEGRO5_LIBRARIES} ${ALLEGRO_PRIMITIVES_LIBRARIES}
                              ${ALLEGRO_TTF_LIBRARIES})


                              
