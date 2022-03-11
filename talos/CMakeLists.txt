# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(pass_box_between_hands
  LAUNCHFILES
  geometric_simu.launch
  gazebo.launch
  on_pyrene.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(pickup_object
  LAUNCHFILES
  geometric_simu.launch
  gazebo.launch
  on_pyrene_no_vision.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(pickup_cardboard_box
  LAUNCHFILES
  geometric_simu.launch
  gazebo.launch
  on_pyrene_no_vision.launch
  on_pyrene.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(manipulate_boxes
  LAUNCHFILES
  gazebo.launch
  estimation.launch
  on_pyrene.launch
  geometric_simu.launch
  world_setup.launch
  support_calibration.launch
  )

# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(camera_calibration
  LAUNCHFILES
  on_pyrene.launch
  )

# Install talos/calibration/apriltags launch files
FOREACH(file demo tracker on_pyrene)
  INSTALL(FILES
    calibration/apriltags/${file}.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    RENAME talos_calibration_apriltags_${file}.launch
    )
ENDFOREACH()

# Install talos/calibration/contact launch files
FOREACH(file bringup_minimal demo simulation world_setup)
  INSTALL(FILES
    calibration/contact/${file}.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    RENAME talos_calibration_contact_${file}.launch
    )
ENDFOREACH()
