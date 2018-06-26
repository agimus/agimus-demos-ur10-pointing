# Theses variables are defined:
# - robot, a SoT device
# - simulateTorqueFeedbackForEndEffector, a boolean

import sys
if not hasattr(sys, "argv"):
    sys.argv = []
import rospy

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
    srdfBox   = parse_srdf ("srdf/cup.srdf", packageName = "sot_hpp_demo", prefix="box")
    for w in [ "grippers", "handles" ]:
        srdf[w] = dict()
        for d in [ srdfTalos, srdfBox ]:
            srdf[w].update (d[w])

    supervisor = Supervisor (robot, hpTasks = hpTasks(robot))
    factory = Factory(supervisor)
    factory.parameters["period"] = rospy.get_param("/sot_controller/dt")
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = True
    # factory.parameters["addTimerToSotControl"] = True
    factory.setGrippers (grippers)
    factory.setObjects (objects, handlesPerObjects, [ [] for e in objects ])
    factory.setRules (rules)
    factory.setupFrames (srdf["grippers"], srdf["handles"], robot)
    # Left gripper
    # Use admittance control when closing
    factory.addAffordance (
        Affordance ("talos/left_gripper", "box/top",
            openControlType="position_torque", closeControlType="position_torque",
            refs = { "angle_open": (0,), "angle_close": (-0.5,), "torque": (-5.,) },
            simuParams = { "refPos": (-0.2,) }))
    # Use position control for opening
    factory.addAffordance (
        Affordance ("talos/left_gripper", None,
            openControlType="position", closeControlType="position",
            refs = { "angle_open": (0,), "angle_close": (-0.5,), }))
    # Right gripper
    # Use position control
    factory.addAffordance (
        Affordance ("talos/right_gripper", "box/bottom",
            openControlType="position", closeControlType="position",
            refs = { "angle_open": (0,), "angle_close": (-0.2,) }))
    factory.generate ()

    supervisor.makeInitialSot ()
    return supervisor

# Set initial config
robot.device.set (tuple([-0.74,] + list(robot.device.state.value[1:])))
supervisor = makeSupervisorWithFactory (robot)

supervisor.plugTopicsToRos()
supervisor.setupEvents ()
supervisor.plugSot("")
