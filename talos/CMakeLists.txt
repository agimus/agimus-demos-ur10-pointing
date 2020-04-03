# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(pass_box_between_hands
  geometric_simu.launch
  gazebo.launch
  on_pyrene.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(pickup_object
  geometric_simu.launch
  gazebo.launch
  on_pyrene_no_vision.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(pickup_cardboard_box
  geometric_simu.launch
  gazebo.launch
  on_pyrene_no_vision.launch
  on_pyrene.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(manipulate_boxes
  gazebo.launch
  estimation.launch
  on_pyrene.launch
  geometric_simu.launch
  world_setup.launch
  support_calibration.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(camera_calibration
  on_pyrene.launch
  )
