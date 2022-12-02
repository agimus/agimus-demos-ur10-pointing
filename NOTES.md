# Notes on the demonstration UR-10 pointing

## Gazebo simulation

1. When plugging the reference velocity in the posture task, the robot
   overshoots the goal configuration. This is due to the difference between
   the assumed (100 Hz) and the actual (143 Hz) frequencies of the stack of
   tasks.

2. To fix 1., the control period specification has been added in the xacro
  file: `<controlPeriod> 0.01 </controlPeriod>`. After this fix, following a
  planned motion with 0 gain in the SoT (testing the reference velocity),
  we observe that the robot still overshoots the reference trajectory. The
  gap seems to appear at the beginning of the motion and then the error
  remains approximately constant.

3. After the end of a motion, the task 'posture_keep' seems to take as the
   reference value the position of the arm at the end of the motion. It would
   better if it took the reference position of the last trajectory.