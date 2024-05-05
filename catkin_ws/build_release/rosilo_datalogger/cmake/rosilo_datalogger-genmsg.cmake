# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rosilo_datalogger: 1 messages, 1 services")

set(MSG_I_FLAGS "-Irosilo_datalogger:/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rosilo_datalogger_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg" NAME_WE)
add_custom_target(_rosilo_datalogger_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rosilo_datalogger" "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg" ""
)

get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv" NAME_WE)
add_custom_target(_rosilo_datalogger_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rosilo_datalogger" "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rosilo_datalogger
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosilo_datalogger
)

### Generating Services
_generate_srv_cpp(rosilo_datalogger
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosilo_datalogger
)

### Generating Module File
_generate_module_cpp(rosilo_datalogger
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosilo_datalogger
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rosilo_datalogger_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rosilo_datalogger_generate_messages rosilo_datalogger_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg" NAME_WE)
add_dependencies(rosilo_datalogger_generate_messages_cpp _rosilo_datalogger_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv" NAME_WE)
add_dependencies(rosilo_datalogger_generate_messages_cpp _rosilo_datalogger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosilo_datalogger_gencpp)
add_dependencies(rosilo_datalogger_gencpp rosilo_datalogger_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosilo_datalogger_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rosilo_datalogger
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosilo_datalogger
)

### Generating Services
_generate_srv_eus(rosilo_datalogger
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosilo_datalogger
)

### Generating Module File
_generate_module_eus(rosilo_datalogger
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosilo_datalogger
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rosilo_datalogger_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rosilo_datalogger_generate_messages rosilo_datalogger_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg" NAME_WE)
add_dependencies(rosilo_datalogger_generate_messages_eus _rosilo_datalogger_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv" NAME_WE)
add_dependencies(rosilo_datalogger_generate_messages_eus _rosilo_datalogger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosilo_datalogger_geneus)
add_dependencies(rosilo_datalogger_geneus rosilo_datalogger_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosilo_datalogger_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rosilo_datalogger
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosilo_datalogger
)

### Generating Services
_generate_srv_lisp(rosilo_datalogger
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosilo_datalogger
)

### Generating Module File
_generate_module_lisp(rosilo_datalogger
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosilo_datalogger
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rosilo_datalogger_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rosilo_datalogger_generate_messages rosilo_datalogger_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg" NAME_WE)
add_dependencies(rosilo_datalogger_generate_messages_lisp _rosilo_datalogger_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv" NAME_WE)
add_dependencies(rosilo_datalogger_generate_messages_lisp _rosilo_datalogger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosilo_datalogger_genlisp)
add_dependencies(rosilo_datalogger_genlisp rosilo_datalogger_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosilo_datalogger_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rosilo_datalogger
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosilo_datalogger
)

### Generating Services
_generate_srv_nodejs(rosilo_datalogger
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosilo_datalogger
)

### Generating Module File
_generate_module_nodejs(rosilo_datalogger
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosilo_datalogger
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rosilo_datalogger_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rosilo_datalogger_generate_messages rosilo_datalogger_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg" NAME_WE)
add_dependencies(rosilo_datalogger_generate_messages_nodejs _rosilo_datalogger_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv" NAME_WE)
add_dependencies(rosilo_datalogger_generate_messages_nodejs _rosilo_datalogger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosilo_datalogger_gennodejs)
add_dependencies(rosilo_datalogger_gennodejs rosilo_datalogger_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosilo_datalogger_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rosilo_datalogger
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_datalogger
)

### Generating Services
_generate_srv_py(rosilo_datalogger
  "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_datalogger
)

### Generating Module File
_generate_module_py(rosilo_datalogger
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_datalogger
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rosilo_datalogger_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rosilo_datalogger_generate_messages rosilo_datalogger_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/msg/AddValueMsg.msg" NAME_WE)
add_dependencies(rosilo_datalogger_generate_messages_py _rosilo_datalogger_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_datalogger/srv/Save.srv" NAME_WE)
add_dependencies(rosilo_datalogger_generate_messages_py _rosilo_datalogger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosilo_datalogger_genpy)
add_dependencies(rosilo_datalogger_genpy rosilo_datalogger_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosilo_datalogger_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosilo_datalogger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosilo_datalogger
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rosilo_datalogger_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosilo_datalogger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rosilo_datalogger
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rosilo_datalogger_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosilo_datalogger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosilo_datalogger
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rosilo_datalogger_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosilo_datalogger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rosilo_datalogger
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rosilo_datalogger_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_datalogger)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_datalogger\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_datalogger
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_datalogger")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosilo_datalogger
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rosilo_datalogger_generate_messages_py std_msgs_generate_messages_py)
endif()
