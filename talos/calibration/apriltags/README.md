# Calibration procedure on Pyrene using april tags on the wrists

This directory contains data and script to calibrate Pyrene robot using
  - April tags attached to the wrist, and
  - the RGB-D camera mounted on the head.

### Data

Directory data contains configurations for which, the left or right wrist is
visible from the camera.

  * File "all-configurations.csv" is ordered using a naive traveller salesman
    solution.
  * "measures-simulation.csv" contains configurations and the
    corresponding measures (positions of left or right support in
    camera coordinate frame). These data have been obtained in
    simulation, but contain outliers.
  * File "measures-offset-arm_left_1.csv" contains simulated data with an
    offset of 0.025 radian on the measure of arm_left_1 encoder.
  * File "one-measure.csv" contains the first measure of the above file.

  * File measurements-pyrene-20200212-1.csv contains measurements obtained on
    Pyrene with 67 configurations. Joint state is read from the SoT.
  * File measurements-pyrene-20200212-2.csv contains measurements obtained on
    Pyrene with 94 configurations. Joint state is read from the robot encoders.

  * File measurements-pyrene-20200819-1.csv contains data acquired on Pyrene
    following the calibration procedure.
  * File measurements-pyrene-20200819-1-filtered.csv contains the above data
    after removing an outlier.

  * Files measurements-pyrene-20200828-1.csv and measurements-pyrene-20200828-2.csv
    have been obtained on Pyrene on configurations reached during the demo
    "Pyrene manipulate a box". The joints angles measured are corrected by the
    previous calibration data (20200819).

### Python scripts

Motion planning, motion generation and calibration computation are controlled
by Python scripts.

  * "script_hpp.py" generates a list of collision-free paths going from the
    first configuration to the last one. Using options, this script can also
    generate such configurations.
  * "play_motion.py" triggers execution of the paths planned by hppcorbaserver.
  * "compute_calibration.py" solves the optimization problem that provide the
    calibration parameters. After running the script, the calibration parameters
    are stored in
      - cc.variable.hTc (SE3) for the position of the camera in 'head_2_joint',
      - cc.variable.lwTls (SE3) for the position of the left support in
        'arm_left_7_joint',
      - cc.variable.rwTrs (SE3) for the position of the right support in
        'arm_right_7_joint', and
      - cc.variable.q_off for the vector of joint offsets in the order
        provided by cc.joints.
    You can use cc.write_xacro() to generate the xacro code that should be put
    into talos_description_calibration/urdf/calibration/calibration_constants.urdf.xacro

### Launch files

  * "demo.launch" run the simulation using ROS. The command is

    <code>roslaunch agimus_demos talos_calibration_apriltags.launch</endcode>
  * "on_pyrene.launch" run the motion execution on the robot. The command is

    <code>roslaunch agimus_demos talos_calibration_apriltags_on_pyrene.launch
    </endcode>
