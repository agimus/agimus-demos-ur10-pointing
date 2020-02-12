# Calibration procedure on Pyrene using april tags on the wrists

This directory contains data and script to calibrate Pyrene robot using
  - April tags attached to the wrist, and
  - the RGB-D camera mounted on the head.

### Data

Directory data contains configurations for which, the left or right wrist is
visible from the camera.

  * File "all-configurations.csv" is ordered using a naive traveller salesman
    solution.
  * "measures.csv" contains configurations and the corresponding measures
    (positions of left or right support in camera coordinate frame). These data
    have been obtained in simulation, but contain outliers.
  * File "one-measure.csv" contains the first measure of the above file.

### Python scripts

Motion planning, motion generation and calibration computation are controlled
by Python scripts.

  * "script_hpp.py" generates a list of collision-free paths going from the
    first configuration to the last one. Using options, this script can also
    generate such configurations.
  * "play_motion.py" triggers execution of the paths planned by hppcorbaserver.
  * "compute_calibration.py" solves the optimization problem that provide the
    calibration parameters.

### Launch files

  * "demo.launch" run the simulation using ROS. The command is

    <code>roslaunch agimus_demos talos_calibration_apriltags.launch</endcode>
  * "on_pyrene.launch" run the motion execution on the robot. The command is

    <code>roslaunch agimus_demos talos_calibration_apriltags_on_pyrene.launch
    </endcode>