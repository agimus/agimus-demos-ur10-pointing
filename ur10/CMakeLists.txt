# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(pointing
  LAUNCHFILES
  simulation.launch
  ur10_upload.launch
  world_setup.launch
  )

INSTALL(FILES
  pointing/supervisor.py
  pointing/demo.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur10/pointing
  )

INSTALL(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur10)
