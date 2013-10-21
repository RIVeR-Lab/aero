cmake_minimum_required(VERSION 2.8.3)
project(aero_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  controller_manager
  hardware_interface 
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  gazebo_ros_control)

find_package(gazebo REQUIRED)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS
      roscpp
      gazebo_ros
      controller_manager
      pluginlib
      transmission_interface
      gazebo_ros_control
    INCLUDE_DIRS
    LIBRARIES gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(aero_hw_sim src/aero_hw_sim.cpp)
target_link_libraries(aero_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

foreach(dir worlds launch)
   install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
install(FILES aero_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS aero_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

