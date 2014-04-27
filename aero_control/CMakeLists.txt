cmake_minimum_required(VERSION 2.8.3)
project(aero_control)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aero_control
  CATKIN_DEPENDS ros_control ros_controllers controller_manager robot_state_publisher robot_drive_controllers joint_state_controller safety_controller
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir config launch)
   install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
