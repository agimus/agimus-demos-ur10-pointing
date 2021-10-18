# Development instructions

These instructions are meant to help run the demo in simulation in the state as
it is. They will evolve and eventually be replaced by instructions to run the
demonstration on the real robot.

## Steps to run the demo

Open several tabs in a terminal, go to directory
`/root/catkin_ws/src/agimus-demos/ur10/pointing` and type the following
instructions

1. in terminal 1
```bash
hppcorbaserver
```

2. in terminal 2
```bash
ipython -i script_hpp.py
```

3. in terminal 3
```bash
gepetto-gui
```

### Generate a trajectory from the initial configuration

4. in terminal 4
```bash
roslaunch ./simulation.launch
```

5. in terminal 2
```python
from tools_hpp import ConfigGenerator, RosInterface

ri = RosInterface(robot)
cg = ConfigGenerator(graph)
q_init = ri.getCurrentConfig(q0)
res = False
while not res:
    res, q_goal, qg = cg.generateValidConfigForHandle('part/handle_0', q0)

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)
ps.solve()
```

6. in terminal 5
```bash
roslaunch ./demo.launch 
```
7. in terminal 6
``` bash
rostopic pub /hpp/target/position dynamic_graph_bridge_msgs/Vector [0,0,0,0,0,0]
```

8. in terminal 7
``` bash
rostopic pub /hpp/target/velocity dynamic_graph_bridge_msgs/Vector [0,0,0,0,0,0]
```

9. in terminal 8
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


10. in terminal 6
stop rostopic pub commands in terminal 6 and 7, then
```bash
rosrun agimus rqt_path_execution 
```
then select path 2 and type execute path.

11. in terminal 8, once the path is executed;
```
python
robot.stopTracer()
```
