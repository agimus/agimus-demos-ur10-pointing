# Calibration

This file explains how to plan, execute motions that move the camera in front
of a chessboard, and how to collect calibration data. We then propose two
calibration procedure

  - hand eye based with VIsP to compute the camera pose in the end-effector
    frame,
  - full calibration based on VIsP and figaroh that besides the above computes
    the geometric parameters of the kinematic chain of the robot.

Before starting, place a chessboard of 10 by 7 squares of size 2.7cm
on a vertical plane in front of the robot (1.3m ahead). The center of
the chessboardat should be at a height of 1.41m.

Remove the wooden spike since the chessboard should not be occluded at all.

## Generating and executing motions

Start the demo with the following differences:

  - no need to start react_inria node,
  - roslaunch demo part:=april-tag-plank,
  - in script_hpp.py, set "UseAprilTagPlank = True",

Move the robot to the "calib" configuration.

In the terminal where script_hpp.py has been run, copy paste the
following lines. 

```python
from calibration import Calibration, generateDataForFigaroh, checkData

calibration = Calibration(ps, graph, factory)
calibration.nbConfigs = 30
calibration.addStateToConstraintGraph()
calibration.generateConfigurationsAndPaths(q0, filename="./data/calib-configs.csv")
```

If no path appears in the "Path Player" of gepetto-gui after
refreshing, re-execute the last line of the later script after
increasing "calibration.nbConfigs".

in a new terminal, cd into ur10/pointing and run
```python
python -i play_path.py
```

In the same terminal, run
```python
playAllPaths(0)
```
The robot should execute the paths and collect the required data at each end
of path. After the last path has been executed,

```bash
mkdir measurements
```
and run
```python
cc.save('./measurements')
```

The procedure can be run several times if not enough data has been
acquired the first time. For that, it is recommended

  - to go back to the "calib" configuration,
  - to save the data in another directory to avoid overwriting the previous data.

## Hand eye calibration

Follow the instructions here https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic.html.

### Updating the model

The calibration procedure computes the pose of the optical frame
"camera_color_optical_frame" in the end-effector link, here "ref_camera_link".
The problem is that this transformation is hard-coded in the camera model.
We therefore need to compute a new transform between "ur10e_d435_mount_link"
and "ref_camera_link" in such a way that the pose of the
"camera_color_optical_frame" in "ur10e_d435_mount_link" is correct. For that,
we denote by

  - m ur10e_d435_mount_link,
  - c camera_color_optical_frame,
  - e ref_camera_link (end effector).

The desired value of mMc is given by:

  mMc_desired = mMe * eMc_measured

where mMe is the pose of "ref_camera_link" in "ur10e_d435_mount_link".
eMc_measured is the result of hand eye calibration. As explained above,
eMc is provided by the camera urdf model that we do not want to modify. We
therefore need to modify mMe into mMe_new to get the same mMc_desired:

  mMc_desired = mMe_new * eMc_provided

Therefore

  mMe_new = mMe * eMc_measured * eMc_provided.inverse()

Function computeCameraPose in calibration.py makes this calculation.

## Full calibration

Install figaroh

```bash
cd $DEVEL_HPP_DIR/src
make figaroh.install
```
### Compute the camera poses with respect to the chessboard:

$DEVEL_HPP_DIR/src/visp/build-rel/tutorial/calibration/tutorial-chessboard-pose --square_size 0.027 --input image-%d.png --intrinsic camera.xml --output pose_cPo_%d.yaml

See  https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic.html for details.

In the python terminal where `script_hpp.py` has been executed, type
```python
generateDataForFigaroh(robot, './measurements', output_file, 30)
```
where `output_file` is the full path to
`examples/ur10/data/calibration.csv` in `figaroh` source directory.

### Compute the kinematic parameters

```bash
cd $DEVEL_HPP_DIR/src/figaroh/examples/ur10
python -i calibration.py
```
Several windows appear that show the result of the optimization procedure. Close
the windows by typing 'q'.
Then in the python terminal, type
```python
from update_model import update_parameters
update_parameters(f_input, f_output, LM_solve.x, param)
```
where

  - `f_input` is the full path to the current kinematic parameters of the
     robot: `$DEVEL_HPP_DIR/src/agimus-demos/ur10/pointing/config/calibrated-params.yaml`,
  - `f_output` is the name of a file in the same directory, for example
    `updated-params.yaml`.
Compare the two files to check that the differences are relevant. If so, copy
the later into the former, commit, push and reinstall agimus-demos.

### Compute the camera pose

The last 6 components of `LM_solve.x` represent the pose of
`camera_color_optical_frame` in `wrist_3_link`: (x,y,z,roll,pitch,yaw).
