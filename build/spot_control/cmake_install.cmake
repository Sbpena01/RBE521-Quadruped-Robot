# Install script for directory: /home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/src/spot_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spot_control/srv" TYPE FILE FILES
    "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/src/spot_control/srv/multiplier.srv"
    "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/src/spot_control/srv/jointControl.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spot_control/cmake" TYPE FILE FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/build/spot_control/catkin_generated/installspace/spot_control-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/devel/include/spot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/devel/share/roseus/ros/spot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/devel/share/common-lisp/ros/spot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/devel/share/gennodejs/ros/spot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/devel/lib/python3/dist-packages/spot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/devel/lib/python3/dist-packages/spot_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/build/spot_control/catkin_generated/installspace/spot_control.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spot_control/cmake" TYPE FILE FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/build/spot_control/catkin_generated/installspace/spot_control-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spot_control/cmake" TYPE FILE FILES
    "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/build/spot_control/catkin_generated/installspace/spot_controlConfig.cmake"
    "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/build/spot_control/catkin_generated/installspace/spot_controlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spot_control" TYPE FILE FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/src/spot_control/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/spot_control" TYPE PROGRAM FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/build/spot_control/catkin_generated/installspace/joint_control_server.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/spot_control" TYPE PROGRAM FILES "/home/nddixon/RBE_Grad/Legged_Robotics/project/RBE521-Quadruped-Robot/build/spot_control/catkin_generated/installspace/joint_control_client.py")
endif()
