# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(pointing
  LAUNCHFILES
  simulation.launch
  ur10_upload.launch
  )

INSTALL(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur10)
