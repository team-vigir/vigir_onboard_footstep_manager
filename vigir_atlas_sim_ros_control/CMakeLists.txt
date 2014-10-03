cmake_minimum_required(VERSION 2.8.3)
project(vigir_atlas_sim_ros_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs hardware_interface controller_manager trajectory_msgs osrf_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vigir_atlas_sim_ros_control
  CATKIN_DEPENDS roscpp std_msgs hardware_interface controller_manager trajectory_msgs osrf_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(atlas_sim_ros_control_node src/atlas_sim_ros_control.cpp)

target_link_libraries(atlas_sim_ros_control_node
  ${catkin_LIBRARIES}
)

add_dependencies(atlas_sim_ros_control_node ${catkin_EXPORTED_TARGETS})