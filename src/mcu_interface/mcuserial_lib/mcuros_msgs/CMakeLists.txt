cmake_minimum_required(VERSION 3.7.2)
project(mcuros_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package()

catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#catkin_install_python(
#  PROGRAMS src/mcuros_msgs/McuToRosMsg.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)