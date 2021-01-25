# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(insert_drill
  LAUNCHFILES
  estimation.launch
  gazebo.launch
  geometric_simu.launch
  tiago.launch
  tiago_hpp.launch
  world_setup.launch
  )

INSTALL_DEMO_LAUNCH_FILES(deburring
  LAUNCHFILES
  2d_navigation.launch
  world_setup.launch
  estimation.launch

  gazebo.launch

  SCRIPTS
  publish_initial_pose.py
  )

find_package(pal_navigation_cfg_tiago)
if (pal_navigation_cfg_tiago_FOUND)
  set(pal_navigation_cfg_tiago_SHARE ${pal_navigation_cfg_tiago_PREFIX}/share/pal_navigation_cfg_tiago)
  install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tiago/config/base/\")")
  foreach(subdir "common" "global_planner")
    set(filepath "${pal_navigation_cfg_tiago_SHARE}/config/base/${subdir}")
    install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \"${filepath}\" \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tiago/config/base/${subdir}\")")
  endforeach()
  install(FILES common/teb/local_planner.yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tiago/config/base/teb)
endif ()
