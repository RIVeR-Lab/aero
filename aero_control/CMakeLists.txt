cmake_minimum_required(VERSION 2.8.3)
project(aero_control)

find_package(catkin REQUIRED COMPONENTS roscpp filters sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aero_control
  CATKIN_DEPENDS ros_control ros_controllers controller_manager robot_state_publisher robot_drive_controllers joint_state_controller safety_controller roscpp filters sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(laser_shutter_filter
	src/laser_shutter_filter.cpp)
target_link_libraries(laser_shutter_filter
  ${catkin_LIBRARIES}
)
add_library(laser_small_object_filter
	src/laser_small_object_filter.cpp)
target_link_libraries(laser_small_object_filter
  ${catkin_LIBRARIES}
)

foreach(dir config launch)
   install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
