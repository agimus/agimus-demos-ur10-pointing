# Instruction to run the calibration demo

These instructions are meant to help run the demo in simulation or an a real
robot.

## In simulation

Open a terminal with several tabs and go into directory ~/catkin_ws/src/agimus-demos/talos/calibration/contact.

1. in terminal 1
```
roslaunch simulation.launch pal_simulator:=true
```
If you drop option `pal_simulator`, gazebo is used instead of PAL physics simulator

2. in terminal 2 (only with PAL simulator)
```
roslaunch talos_controller_configuration default_controllers.launch
```

In Gazebo, the simulation is paused at startup. Click on button play in gazebo window to
start it.

2. in terminal 3
```
hppcorbaserver
```

3. in terminal 4
```
ipython -i script_hpp.py
```

4. in terminal 5
```
gepetto-gui
```

5. in terminal 4
```
v=vf.createViewer()
q_init = ri.getCurrentConfig(initConf, 5., 'talos/leg_left_6_joint')
v(q_init)

from agimus_demos.tools_hpp import PathGenerator
pg = PathGenerator(ps,graph)
pg.inStatePlanner.maxIterPathPlanning = 100
goToContact(ri, pg, 'talos/left_gripper', 'table/contact_01', q_init)

```
In the path player of `gepetto-gui`, you should see three paths ready to be executed.

6. In terminal 2

If any, stop the current roslaunch
```
roslaunch demo.launch simulation:=true
```

7. in terminal 6
```
rosrun agimus rqt_path_execution
```
Select the path to execute and click on "Execute Path".

### Record the traces

9. in terminal 7
``` bash
rostopic pub /hpp/target/position dynamic_graph_bridge_msgs/Vector [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
```

10. in terminal 8
``` bash
rostopic pub /hpp/target/velocity dynamic_graph_bridge_msgs/Vector [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
```

11. in terminal 9
```bash
rosrun dynamic_graph_bridge run_command
```
then
```python
from dynamic_graph import factory_get_entity_class_list, get_entity_list

entities = get_entity_list()
cas = list()
for e in entities:
    try:
        cas.append(ContactAdmittance(e))
    except:
        pass

ca = cas[0]
ca.getWrenchOffset(robot.device.control.time)
robot.initializeTracer()
robot.tracer.setBufferSize(2**24)
robot.addTrace(cas[0].name, 'contact')
robot.addTrace(cas[0].name, 'error')
robot.addTrace(cas[0].name, 'errorIn')
robot.addTrace(cas[0].name, 'wrench')
robot.addTrace(cas[0].name, 'releaseCriterion')
robot.addTrace(cas[0].name, 'wrenchMinusOffset')

robot.startTracer()
```

## On the robot

Make sure that all terminals in the console and on the robot have the following
environment variables set:

export HPP_HOST=10.68.1.11
export HPP_PORT=13331

1. on talos-1c, go to halfsitting

```
rosrun talos_controller_configuration talos_initialisation.py --wrist -y
```

2. on talos-1m

```
hppcorbaserver
```

3. on the console
```
gepetto-gui
```

4. on the console
```
ipython -i script_hpp.py
```

5. then in the python terminal
```
v=vf.createViewer(host="10.68.1.38")
q_init = ri.getCurrentConfig(initConf, 5., 'talos/leg_left_6_joint')
v(q_init)

from agimus_demos.tools_hpp import PathGenerator
pg = PathGenerator(ps,graph)
pg.inStatePlanner.maxIterPathPlanning = 100
goToContact(ri, pg, 'talos/left_gripper', 'table/contact_01', q_init)
```

6. on talos-1c, launch the demonstration

```
roslaunch agimus_demos talos_calibration_contact_demo.launch
```

Follow the same steps as in simulation starting by 7.
