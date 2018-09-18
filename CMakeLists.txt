cmake_minimum_required(VERSION 2.8.3)
project(agimus_demos)

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
#  LIBRARIES agimus_demos
#  CATKIN_DEPENDS roslaunch
#  DEPENDS system_lib
)

MACRO(INSTALL_DEMO_LAUNCH_FILES ROBOT DEMO)
  SET (LAUNCH_FILES ${ARGN})
  FOREACH (launch_file ${LAUNCH_FILES})
    install(FILES
      ${ROBOT}/${DEMO}/${launch_file}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
      RENAME ${ROBOT}_${DEMO}_${launch_file}
    )
  ENDFOREACH ()
ENDMACRO()

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(talos pass_box_between_hands
  geometric_simu.launch
  gazebo.launch
  on_pyrene.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(talos pickup_object
  geometric_simu.launch
  gazebo.launch
  on_pyrene_no_vision.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(talos pickup_cardboard_box
  geometric_simu.launch
  gazebo.launch
  on_pyrene_no_vision.launch
  on_pyrene.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(talos manipulate_boxes
  estimation.launch
  on_pyrene.launch
  )

# Install other files with the same hierarchy as here.
install(DIRECTORY talos tiago urdf srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*"
  PATTERN "*.launch" EXCLUDE
)

install(PROGRAMS
  scripts/gazebo_object_to_tf.py
  scripts/visual_tag_initializer.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
