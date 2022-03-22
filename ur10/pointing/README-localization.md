# Launch the camera node

0. In terminal 1, run the following command to start realsense with ros, with the option to publish the pointcloud:
```bash
roslaunch realsense2_camera rs_aligned_depth.launch filters:=pointcloud
```

Check that the package ```react_inria``` is installed and go to the folder ```react_inria/launch```.

# Running react_inria localization node in simulation

1. To provide the initial pose of the part in simulation, in terminal 2, while in the  ```react_inria/launch``` folder:

```bash
roslaunch ./react_learning.launch use_sim_time:=true
```
And follow the instructions to generate a new pose and save it.

2. Run the localization node in simulation, in terminal 2:

```bash
roslaunch ./react_localizer.launch use_wMo_filename:=$HOME/.ros/transform_wMo.yaml use_oMo_filename:=$HOME/.ros/transform_oMo.yaml use_aligned_depth:=false use_sim_time:=true
```

3. In terminal 3, launch the script publishing the frame used in the demo, in `agimus-demos/script` folder:

```bash
./publish_frames.py 
```

# Running react_inria localization node on the robot

1. To provide the initial pose of the part on the real robot, in terminal 2, while in the  ```react_inria/launch``` folder:

```bash
roslaunch ./react_learning.launch
```
And follow the instructions to generate a new pose and save it. This needs to be done every time the part is moved in regards to the robot base.

2. Run the localization node in simulation, in terminal 2:
```bash
roslaunch ./react_localizer.launch use_wMo_filename:=$HOME/.ros/transform_wMo.yaml use_oMo_filename:=$HOME/.ros/transform_oMo.yaml use_aligned_depth:=true
```

To use the neural network version of the localizer :
```bash
roslaunch ./react_localizer.launch use_wMo_filename:=$HOME/.ros/transform_wMo.yaml use_oMo_filename:=$HOME/.ros/transform_oMo.yaml use_dnn_localizer:=true
```

3. In terminal 3, launch the script publishing the frame used in the demo, in `agimus-demos/script` folder:

```bash
./publish_frames.py 
```