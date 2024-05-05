# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rosilo_polaris_vega: 1 messages, 0 services")

set(MSG_I_FLAGS "-Irosilo_polaris_vega:/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rosilo_polaris_vega_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg" NAME_WE)
add_custom_target(_rosilo_polaris_vega_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rosilo_polaris_vega" "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rosilo_polaris_vega
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosilo_polaris_vega
)

### Generating Services

### Generating Module File
_generate_module_cpp(rosilo_polaris_vega
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosilo_polaris_vega
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rosilo_polaris_vega_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rosilo_polaris_vega_generate_messages rosilo_polaris_vega_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg" NAME_WE)
add_dependencies(rosilo_polaris_vega_generate_messages_cpp _rosilo_polaris_vega_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosilo_polaris_vega_gencpp)
add_dependencies(rosilo_polaris_vega_gencpp rosilo_polaris_vega_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosilo_polaris_vega_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rosilo_polaris_vega
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosilo_polaris_vega
)

### Generating Services

### Generating Module File
_generate_module_eus(rosilo_polaris_vega
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosilo_polaris_vega
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rosilo_polaris_vega_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rosilo_polaris_vega_generate_messages rosilo_polaris_vega_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg" NAME_WE)
add_dependencies(rosilo_polaris_vega_generate_messages_eus _rosilo_polaris_vega_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosilo_polaris_vega_geneus)
add_dependencies(rosilo_polaris_vega_geneus rosilo_polaris_vega_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosilo_polaris_vega_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rosilo_polaris_vega
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosilo_polaris_vega
)

### Generating Services

### Generating Module File
_generate_module_lisp(rosilo_polaris_vega
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosilo_polaris_vega
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rosilo_polaris_vega_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rosilo_polaris_vega_generate_messages rosilo_polaris_vega_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg" NAME_WE)
add_dependencies(rosilo_polaris_vega_generate_messages_lisp _rosilo_polaris_vega_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosilo_polaris_vega_genlisp)
add_dependencies(rosilo_polaris_vega_genlisp rosilo_polaris_vega_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosilo_polaris_vega_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rosilo_polaris_vega
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosilo_polaris_vega
)

### Generating Services

### Generating Module File
_generate_module_nodejs(rosilo_polaris_vega
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosilo_polaris_vega
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rosilo_polaris_vega_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rosilo_polaris_vega_generate_messages rosilo_polaris_vega_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg" NAME_WE)
add_dependencies(rosilo_polaris_vega_generate_messages_nodejs _rosilo_polaris_vega_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosilo_polaris_vega_gennodejs)
add_dependencies(rosilo_polaris_vega_gennodejs rosilo_polaris_vega_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosilo_polaris_vega_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rosilo_polaris_vega
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_polaris_vega
)

### Generating Services

### Generating Module File
_generate_module_py(rosilo_polaris_vega
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_polaris_vega
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rosilo_polaris_vega_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rosilo_polaris_vega_generate_messages rosilo_polaris_vega_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_polaris_vega/msg/ToolsPoseArray.msg" NAME_WE)
add_dependencies(rosilo_polaris_vega_generate_messages_py _rosilo_polaris_vega_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosilo_polaris_vega_genpy)
add_dependencies(rosilo_polaris_vega_genpy rosilo_polaris_vega_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosilo_polaris_vega_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosilo_polaris_vega)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosilo_polaris_vega
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rosilo_polaris_vega_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rosilo_polaris_vega_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosilo_polaris_vega)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosilo_polaris_vega
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rosilo_polaris_vega_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(rosilo_polaris_vega_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosilo_polaris_vega)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosilo_polaris_vega
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rosilo_polaris_vega_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(rosilo_polaris_vega_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosilo_polaris_vega)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosilo_polaris_vega
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rosilo_polaris_vega_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(rosilo_polaris_vega_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_polaris_vega)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_polaris_vega\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_polaris_vega
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rosilo_polaris_vega_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rosilo_polaris_vega_generate_messages_py geometry_msgs_generate_messages_py)
endif()
