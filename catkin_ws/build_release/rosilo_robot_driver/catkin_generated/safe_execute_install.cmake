execute_process(COMMAND "/home/yuki/git/ctyuen2022/catkin_ws/build_release/rosilo_robot_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/yuki/git/ctyuen2022/catkin_ws/build_release/rosilo_robot_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
