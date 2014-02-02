# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(aero_description)
find_package(catkin REQUIRED)


catkin_package(CATKIN_DEPENDS xacro husky_description)

foreach(dir meshes urdf robots launch)
   install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
