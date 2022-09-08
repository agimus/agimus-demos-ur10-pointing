# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(pointing
  LAUNCHFILES
  demo.launch
  estimation.launch
  estimation_realsense.launch
  load_ur.launch
  load_ur10e.launch
  simulation.launch
  ur10_upload.launch
  world_setup.launch
  )

INSTALL(FILES
  pointing/supervisor.py
  pointing/demo-plaque-tubes-with-table.yaml
  pointing/demo-april-tag-plank.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur10/pointing
  )
INSTALL_DEMO_LAUNCH_FILES(bin-picking
  LAUNCHFILES
  demo.launch
  load_ur.launch
  load_ur10e.launch
  simulation.launch
  ur10_upload.launch
  world_setup.launch
  )

INSTALL(FILES
  bin-picking/supervisor.py
  bin-picking/demo-bin-picking-part.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur10/bin-picking
  )
INSTALL(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ur10)
