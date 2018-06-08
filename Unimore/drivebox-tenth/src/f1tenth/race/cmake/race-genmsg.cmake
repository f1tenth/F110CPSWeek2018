# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "race: 3 messages, 0 services")

set(MSG_I_FLAGS "-Irace:/home/ubuntu/race/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(race_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/race/msg/drive_param.msg" NAME_WE)
add_custom_target(_race_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "race" "/home/ubuntu/race/msg/drive_param.msg" ""
)

get_filename_component(_filename "/home/ubuntu/race/msg/pid_input.msg" NAME_WE)
add_custom_target(_race_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "race" "/home/ubuntu/race/msg/pid_input.msg" ""
)

get_filename_component(_filename "/home/ubuntu/race/msg/drive_values.msg" NAME_WE)
add_custom_target(_race_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "race" "/home/ubuntu/race/msg/drive_values.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(race
  "/home/ubuntu/race/msg/drive_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race
)
_generate_msg_cpp(race
  "/home/ubuntu/race/msg/pid_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race
)
_generate_msg_cpp(race
  "/home/ubuntu/race/msg/drive_values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race
)

### Generating Services

### Generating Module File
_generate_module_cpp(race
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(race_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(race_generate_messages race_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/race/msg/drive_param.msg" NAME_WE)
add_dependencies(race_generate_messages_cpp _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/race/msg/pid_input.msg" NAME_WE)
add_dependencies(race_generate_messages_cpp _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/race/msg/drive_values.msg" NAME_WE)
add_dependencies(race_generate_messages_cpp _race_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(race_gencpp)
add_dependencies(race_gencpp race_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS race_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(race
  "/home/ubuntu/race/msg/drive_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race
)
_generate_msg_lisp(race
  "/home/ubuntu/race/msg/pid_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race
)
_generate_msg_lisp(race
  "/home/ubuntu/race/msg/drive_values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race
)

### Generating Services

### Generating Module File
_generate_module_lisp(race
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(race_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(race_generate_messages race_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/race/msg/drive_param.msg" NAME_WE)
add_dependencies(race_generate_messages_lisp _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/race/msg/pid_input.msg" NAME_WE)
add_dependencies(race_generate_messages_lisp _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/race/msg/drive_values.msg" NAME_WE)
add_dependencies(race_generate_messages_lisp _race_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(race_genlisp)
add_dependencies(race_genlisp race_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS race_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(race
  "/home/ubuntu/race/msg/drive_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race
)
_generate_msg_py(race
  "/home/ubuntu/race/msg/pid_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race
)
_generate_msg_py(race
  "/home/ubuntu/race/msg/drive_values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race
)

### Generating Services

### Generating Module File
_generate_module_py(race
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(race_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(race_generate_messages race_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/race/msg/drive_param.msg" NAME_WE)
add_dependencies(race_generate_messages_py _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/race/msg/pid_input.msg" NAME_WE)
add_dependencies(race_generate_messages_py _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/race/msg/drive_values.msg" NAME_WE)
add_dependencies(race_generate_messages_py _race_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(race_genpy)
add_dependencies(race_genpy race_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS race_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(race_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(race_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(race_generate_messages_py std_msgs_generate_messages_py)
