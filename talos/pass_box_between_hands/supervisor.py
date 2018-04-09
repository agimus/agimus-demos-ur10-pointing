import sys
if not hasattr(sys, "argv"):
    sys.argv = []

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
    from sot_hpp.srdf_parser import parse_srdf
    from hpp.corbaserver.manipulation import Rule

    grippers = [ "talos/left_gripper", "talos/right_gripper" ]
    objects = [ "box" ]
    handlesPerObjects = [ [ "box/top", "box/bottom" ], ]
    rules = [ Rule([ "talos/left_gripper", ], [ "box/top", ], True),
              Rule([ "talos/right_gripper", ], [ "box/bottom", ], True), ]

    srdf = {}
    srdfTalos = parse_srdf ("srdf/talos.srdf", packageName = "talos_data", prefix="talos")
    # Full path can be provided with
    # srdfBox   = parse_srdf ("cup.srdf")
    srdfBox   = parse_srdf ("srdf/cup.srdf", packageName = "hpp_tutorial", prefix="box")
    for w in [ "grippers", "handles" ]:
        srdf[w] = dict()
        for d in [ srdfTalos, srdfBox ]:
            srdf[w].update (d[w])

    supervisor = Supervisor (robot, hpTasks = hpTasks(robot))
    factory = Factory(supervisor)
    factory.setGrippers (grippers)
    factory.setObjects (objects, handlesPerObjects, [ [] for e in objects ])
    factory.setRules (rules)
    factory.setupFrames (srdf["grippers"], srdf["handles"], robot)
    factory.addAffordance (
        Affordance ("talos/left_gripper", "box/top",
          refOpen=(0,), refClose=(-0.2,)))
    factory.addAffordance (
        Affordance ("talos/right_gripper", "box/bottom",
          refOpen=(0,), refClose=(-0.2,)))
    factory.generate ()

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
