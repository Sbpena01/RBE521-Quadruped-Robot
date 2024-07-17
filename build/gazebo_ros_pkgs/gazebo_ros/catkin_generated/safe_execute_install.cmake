execute_process(COMMAND "/home/sbpena01/GitHub/RBE521-Quadruped-Robot/build/gazebo_ros_pkgs/gazebo_ros/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sbpena01/GitHub/RBE521-Quadruped-Robot/build/gazebo_ros_pkgs/gazebo_ros/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
