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
