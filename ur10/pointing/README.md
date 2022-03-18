# Development instructions

These instructions are meant to help run the demo in simulation or an a real
robot.

## I- Turning the robot on

1. Turn the robot on, click on the red button in the lower left part of the
   tablet, click on "ON", verify that the two first steps on the screen turn
   green. Click on "START". The remaining states should turn green.
2. Load the file ros.urp on the tablet.
3. Verify that the robot is in the home position. The robot can automatically
   move to the position by entering "Home" and hold the button
   "Move robot to New Position".
   Sometime,the robot will go the protective state to prevent collision.
   In that case, rotate the wrist to a safe state and try again.
4. In the Installation tab, check the Safety => Robot Limits section to ensure
   the robot is in the least restricted conifiguration.

to point the ROS_MASTER_URI, HPP_HOST, ROS_IP to the right configuration.
6. The computer connected to robot should have the Ethernet configuration as following
    IPv4 addr: 192.168.56.1
    Netmask  : 255.255.255.0
    Gateway  : 192.168.56.1

7. This needs to be done only once: In the docker add --privileged parameter to enable running realsense camera in docker image
For example:
docker run -it --rm --name ur10e --cap-add NET_ADMIN -v "/dev:/dev" --privileged --net=host gitlab.laas.fr:4567/rob4fam/docker/ur10-pointing:5

Copy the files https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules to /etc/udev/rules.d/ before connecting the camera


## II- 3D models (this needs to be done only once)

For Onshape, an account is needed before downloading stl file

Camera mount: https://cad.onshape.com/documents/e21ee279b60811091f599ced/w/b0798cf871b5308cc07c899f/e/5201c11ca0cbe439c3f7114c

Drill Tip mount: https://cad.onshape.com/documents/c03aa227492bc9ee7d6bcfaf/w/85e5617ae64ca6b7c1ea8b43/e/44cc71f7db254f57f77def39

Wires clamps: https://www.thingiverse.com/thing:3832407

## III- Steps to run the demo

Open several tabs in a terminal, go to directory
`/root/catkin_ws/src/agimus-demos/ur10/pointing` and type the following
instructions

0. Follow the instructions in `README-localization.md` to launch the camera node and the localizer node.

### III-1) Launch the robot driver and controller

1. in terminal 1

In simulation:
```bash
roslaunch ./simulation.launch
```

On the robot instead:
```bash
roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.56.5 robot_description_file:=$DEVEL_HPP_DIR/install/share/agimus_demos/launch/ur10_pointing_load_ur10e.launch
```

If this fails, with the error message `Could not get fresh data package from robot`, turn off and on the robot by clicking on the green button in the lower left of the tablet, then on `OFF`, then `ON` then `START` and try to run the robot bringup again.

After ROS is launched successfully, press the "play" icon in the tablet
and select "Play from beginning Robot Program". In the terminal you should
read
```bash
[ INFO] [1634638700.530389390]: Robot connected to reverse interface. Ready to receive control commands.
```

2. After press "play", open new terminal (terminal 2) at `/agimus-demos/ur10/pointing` and run:
```bash
python stop-controllers.py
```

3. in terminal 2:
```bash
roslaunch ./demo.launch simulation:=true
```
or on the real robot:
```bash
roslaunch ./demo.launch
```
On the robot, after the file is run, the robot should have a lock sound of the joint.

### III-2) Launch HPP 

4. In terminal 3
```bash
hppcorbaserver
```

5. In terminal 4
```bash
gepetto-gui
```

6. In terminal 5
```bash
ipython -i script_hpp.py
```

### III-3) Launch the vision module

Follow the instructions in ```README-localization.md``` to launch the INRIA REACT vision node.

### III-4) Executing a path

Once a path has been computed using HPP, to execute it, the path ID must be sent through the ros topic "/agimus/start_path". This can also be done one of the following way :

1. With the widhet rqt_path_execution. In a new terminal (terminal 6), run
```bash
rosrun agimus rqt_path_execution
```

In the widget window, you can select the path id. The ```level``` value indicates how much the path will be cut into steps. If ```level=0``` then the path is executed in one step. If ```level>0```, the path is cut into several steps. To execute each step, click ```Execute step``` until the path is completely executed.

2. In the script terminal (terminal 5), run
```python
pg.demo_execute(pid, steps=False)
```

where pid is the path id. If ```steps=True```, the path will be displayed in the GUI and the script will wait for user input before executing the path.

### III-5) Run the demo

1. In the script terminal (terminal 5), run:
```python
pg.demo(hole_list=[1,2,3], steps=True)
```

The robot will point each hole specified in hole_list, in sequence. If ```steps=True```, then each path computed will be displayed in the GUI and the script will wait for user input before executing each path. If ```steps=False```, the demo will run without interruption. If there is a problem and the robot must be stopped, use either the red emergency button on the pendant, or the button "Trigger event error" in the rqt_path_execution widget.

## IV- Record the traces

1. In terminal 7
``` bash
rostopic pub /hpp/target/position dynamic_graph_bridge_msgs/Vector [0,0,0,0,0,0]
```

2. In terminal 8
``` bash
rostopic pub /hpp/target/velocity dynamic_graph_bridge_msgs/Vector [0,0,0,0,0,0]
```

3. In terminal 9
```bash
rosrun dynamic_graph_bridge run_command
```
then
```python
from dynamic_graph.sot.core.task import Task
from dynamic_graph.sot.core.feature_posture import FeaturePosture
from dynamic_graph.sot.universal_robot.sot_universal_robot_device \
    import DeviceUniversalRobot

supervisor.sots["Loop | f"].sot.setMaxControlIncrementSquaredNorm(1e8)
robot.initializeTracer()
robot.tracer.setBufferSize(2**24)
robot.addTrace('___posture_task', 'controlGain')
robot.addTrace('___posture_task', 'task')
robot.addTrace('___posture_task', 'error')
robot.addTrace('___posture_feature_', 'error')
robot.addTrace('___posture_feature_','posture')
robot.addTrace('___posture_feature_','postureDot')
robot.addTrace('___posture_feature_','state')
robot.addTrace('ros_queued_subscribe', 'posture')
robot.addTrace('UniversalRobot', 'state')
robot.addTrace('UniversalRobot', 'robotState')
robot.addTrace('UniversalRobot', 'control')
robot.addTrace('ur_dynamic', 'position')

robot.startTracer()
```

4. Play the path.

5. In terminal 9, once the path is executed:
```
python
robot.stopTracer()
```

## V- Calibrate the end-effector

1. Generate a path to a chosen hole, for example hole 1.  In the script terminal (terminal 5), run:
```python
pid, q = pg.planDeburringPathForHole(1)
```

2. Using the widget rqt_path_execution, with ```level=3```, execute the path step by step until the end-effector is in grasp position.

3. Open the calibration widget. In another terminal, run
```bash
rosrun agimus rqt_tooltip_calibration
```

4. In the calibration widget, select the handle corresponding to the hole (here, 1). Then move the axes x, y, z while observing the position of the tip of the end-effector until it is centered in the hole.

5. Open the file ```agimus-demos/urdf/ur.xacro```. On the lines:
```xml
   <!-- Offset of the tip link (obtained by calibration) -->
   <xacro:arg name="tip_offset_x" default="0.0008"/>
   <xacro:arg name="tip_offset_y" default="0.0069"/>
   <xacro:arg name="tip_offset_z" default="-0.0083"/>
```
Add the value your obtained from the calibration widget to the existing value on those lines.

6. Reinstall agimus-demos and re-run the demo. Since the URDF description is passed through rosparams, you need to restard eveything, including the UR10 bringup.

7. Compute and execute a new pointing movement to check that the calibration has been correctly applied.