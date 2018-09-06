# Theses variables are defined:
# - robot, a SoT device
# - simulateTorqueFeedbackForEndEffector, a boolean

def hpTasks(sotrobot):
    from agimus_sot.tools import COM, Foot, Manifold
    com = COM ("talos", sotrobot)
    lf = Foot ("talos/leg_left_6_joint", sotrobot) # Feet placement make the robot fall down.
    rf = Foot ("talos/leg_right_6_joint", sotrobot)
    return com + lf + rf

def makeSupervisorWithFactory (robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.tools import Manifold
    from agimus_sot.srdf_parser import parse_srdf
    from hpp.corbaserver.manipulation import Rule

    grippers = [ "talos/left_gripper", ]
    objects = [ "box" ]
    handlesPerObjects = [ [ "box/handle1", "box/handle2" ], ]
    contactPerObjects = [ [ "box/bottom_surface", ] ]
    rules = [
              Rule([ "talos/left_gripper", ], [ "box/handle2", ], False),
              # Rule([ "talos/left_gripper", ], [ Object.handles[0], ], True),
              Rule([ "talos/left_gripper", ], [ ".*", ], True),
              # Rule([ "talos/right_gripper", ], [ Object.handles[1], ], True),
              ]

    srdf = {}
    srdfTalos = parse_srdf ("srdf/talos.srdf", packageName = "talos_data", prefix="talos")
    srdfBox   = parse_srdf ("srdf/cobblestone.srdf", packageName = "gerard_bauzil", prefix="box")
    srdfTable = parse_srdf ("srdf/pedestal_table.srdf", packageName = "gerard_bauzil", prefix="table")
    for w in [ "grippers", "handles" ]:
        srdf[w] = dict()
        for d in [ srdfTalos, srdfBox, srdfTable ]:
            srdf[w].update (d[w])

    supervisor = Supervisor (robot, hpTasks = hpTasks(robot))
    factory = Factory(supervisor)
    factory.parameters["period"] = 0.001 # TODO soon: robot.getTimeStep()
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = True
    factory.parameters["useMeasurementOfObjectsPose"] = True
    # factory.parameters["addTimerToSotControl"] = True
    factory.setGrippers (grippers)
    factory.setObjects (objects, handlesPerObjects, contactPerObjects)
    factory.environmentContacts (["table/support",])
    factory.setRules (rules)
    factory.setupFrames (srdf["grippers"], srdf["handles"], robot, disabledGrippers=["table/pose",])
    factory.addAffordance (
        Affordance ("talos/left_gripper", "box/handle1",
            openControlType="torque", closeControlType="torque",
            refs = { "angle_open": (0,), "angle_close": (-0.5,), "torque": (-0.05,) },
            controlParams = { "torque_num": ( 5000., 1000.),
                "torque_denom": (0.01,) },
            simuParams = { "refPos": (-0.2,) }))
    factory.addAffordance (
        Affordance ("talos/left_gripper", None,
            openControlType="position", closeControlType="position",
            refs = { "angle_open": (0,), "angle_close": (-0.5,), "torque": (-0.05,) },
            simuParams = { "refPos": (-0.2,) }))
    factory.generate ()

    supervisor.makeInitialSot ()

    # starting_motion: From half_sitting to position where gaze and COM constraints are satisfied.
    sot_loop =supervisor.sots['Loop | f']
    supervisor.addSot("starting_motion", sot_loop, sot_loop.control)
    return supervisor

# Set initial config
robot.device.set (tuple([-0.6,-0.2,] + list(robot.device.state.value[2:])))
supervisor = makeSupervisorWithFactory (robot)

supervisor.plugTopicsToRos()
supervisor.setupEvents ()
supervisor.plugSot("")
