import sys
if not hasattr(sys, "argv"):
    sys.argv = []

class Client:
    from hpp.corbaserver import Client as BasicClient
    from hpp.corbaserver.manipulation import Client as ManipClient
    def __init__(self):
        self.basic = Client.BasicClient()
        self.manipulation = Client.ManipClient()

def hpTasks(sotrobot):
    from sot_hpp.tools import COM, Foot, Manifold
    com = COM ("talos", sotrobot)
    lf = Foot ("talos/leg_left_6_joint", sotrobot) # Feet placement make the robot fall down.
    rf = Foot ("talos/leg_right_6_joint", sotrobot)
    return com + lf + rf

def makeSupervisorWithFactory (robot):
    from sot_hpp import Supervisor
    from sot_hpp.factory import Factory, Affordance
    from sot_hpp.tools import Manifold
    from hpp.corbaserver.manipulation import Rule

    hppclient = Client()

    grippers = [ "talos/left_gripper", "talos/right_gripper" ]
    objects = [ "box" ]
    handlesPerObjects = [ [ "box/top", "box/bottom" ], ]
    rules = [ Rule([ "talos/left_gripper", ], [ "box/top", ], True),
              Rule([ "talos/right_gripper", ], [ "box/bottom", ], True), ]

    supervisor = Supervisor (robot, hpTasks = hpTasks(robot))
    factory = Factory(supervisor)
    factory.setGrippers (grippers)
    factory.setObjects (objects, handlesPerObjects, [ [] for e in objects ])
    factory.setRules (rules)
    factory.setupFrames (hppclient, robot)
    factory.addAffordance (
        Affordance ("talos/left_gripper", "box/top",
          refOpen=(0,), refClose=(-0.2,)))
    factory.addAffordance (
        Affordance ("talos/right_gripper", "box/bottom",
          refOpen=(0,), refClose=(-0.2,)))
    factory.generate ()
    factory.finalize (hppclient)

    supervisor.makeInitialSot ()
    return supervisor

# Set initial config
robot.device.set (tuple([-0.74,] + list(robot.device.state.value[1:])))
supervisor = makeSupervisorWithFactory (robot)

from dynamic_graph_hpp.sot import RosQueuedSubscribe
re = RosQueuedSubscribe ('ros_export')

supervisor.plugTopics(re)
supervisor.setupEvents ()
supervisor.plugSot("")
