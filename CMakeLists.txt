cmake_minimum_required(VERSION 2.8.3)
project(sot_hpp_demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sot_hpp_demo
#  CATKIN_DEPENDS roslaunch
#  DEPENDS system_lib
)

# Install launch files in launch directory
install(FILES
  talos/pass_box_between_hands/geometric_simu.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  RENAME talos_pass_box_between_hands_geomtric_simu.launch
)
install(FILES
  talos/pass_box_between_hands/gazebo.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  RENAME talos_pass_box_between_hands_gazebo.launch
)
install(FILES
  talos/pass_box_between_hands/on_pyrene.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  RENAME talos_pass_box_between_hands_on_pyrene.launch
)

# Install other files with the same hierarchy as here.
install(DIRECTORY talos tiago urdf srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*"
  PATTERN "*.launch" EXCLUDE
)
