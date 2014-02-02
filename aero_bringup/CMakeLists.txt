cmake_minimum_required(VERSION 2.8.3)
project(aero_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(CATKIN_DEPENDS aero_description aero_control aero_gazebo ros_lm_sensors diagnostic_aggregator mk3_driver robot_teleop)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch)
   install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
