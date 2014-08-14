cmake_minimum_required(VERSION 2.8.3)
project(adept_moveit_connector)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  sensor_msgs
)

catkin_package()

install(PROGRAMS
   scripts/adept_joint_state_publisher.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
