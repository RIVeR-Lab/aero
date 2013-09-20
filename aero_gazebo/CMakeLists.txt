cmake_minimum_required(VERSION 2.8.3)
project(aero_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_ros)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

foreach(dir worlds launch)
   install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
