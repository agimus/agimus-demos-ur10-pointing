# How to generate Rosbags of the UR10 pointing demo in simulation

The rosbags are needed by the vision team to test their software.

## Record the rosbags

Be sure you have `realsense-ros` and `realsense_gazebo_plugin` installed.

### Run the demo

Open several tabs in a terminal, go to directory
`$DEVEL_DIR/agimus-demos/ur10/pointing` and type the following
instructions

1. In terminal 1:
```bash
hppcorbaserver
```

2. In terminal 2:
```bash
ipython -i script_hpp.py
```

3. (Optional) In terminal 3:
```bash
source /root/setup_ld.sh
gepetto-gui
```

4. In terminal 4:
```bash
roslaunch ./simulation.launch simulation:=true
```

5. In terminal 5:
```bash
roslaunch ./demo.launch simulation:=true
```

### Generate the path

6. In terminal 2:
```python
path_ids, grasp_configs = go()
```
The paths are in path_ids. If you want to concatenate all the paths into one :
```python
p_id = concat(path_ids)
```
p_id is then the ID of the global path.

### Play the path and record the bags

7. In terminal 6, start recording the needed topics:
```bash
rosbag record --split --size=1024 /camera/color/image_raw /camera/color/camera_info /camera/depth/camera_info /camera/depth/image_rect_raw /tf /tf_static /pose /camera/extrinsics/depth_to_color
```
This will generate rosbags of 1GB each (the `--size` argumet lets you define the maximum size).

8. In terminal 7:
```bash
rosrun agimus rqt_path_execution
```
then select path p_id (or the path you want to play) and click execute path. The path will be played _very slowly_ so that Gazebo can keep a reasonnable frame rate.

9. Stop the recording in terminal 6. The bags are ready.

## Replay the rosbags

1. In terminal 1, launch rviz:
```bash
roslaunch /root/catkin_ws/src/agimus-demos/ur10/pointing/rviz_rosbag.launch
```

2. In terminal 3, launch roscore:
```bash
roscore
```

3. In terminal 3, play the bags:
```bash
rosbag play --clock my_bag*.bag
```
to play all the bags starting with "my_bag".