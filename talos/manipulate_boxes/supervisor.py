# Theses variables are defined:
# - robot, a SoT device
# - simulateTorqueFeedbackForEndEffector, a boolean

def hpTasks(sotrobot):
    from agimus_sot.tools import COM, Foot, Manifold
    com = COM ("talos", sotrobot)
    lf = Foot ("talos/leg_left_6_joint", sotrobot)
    rf = Foot ("talos/leg_right_6_joint", sotrobot)
    #return com + lf + rf
    #return lf + rf
    #return lf
    #return rf
    #return com
    return Manifold()

def makeSupervisorWithFactory (robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.tools import Manifold
    from agimus_sot.srdf_parser import parse_srdf
    from hpp.corbaserver.manipulation import Rule

    grippers = [ "talos/left_gripper", "talos/right_gripper", ]
    objects = [ "box" ]
    handlesPerObjects = [ [ "box/handle" + str(i) for i in range(1,5) ], ]
    contactPerObjects = [ [ "box/front_surface", "box/rear_surface", ] ]
    rules = [ ]

    srdf = {}
    srdfTalos = parse_srdf ("srdf/talos.srdf", packageName = "talos_data", prefix="talos")
    srdfBox   = parse_srdf ("srdf/plank_of_wood1.srdf", packageName = "gerard_bauzil", prefix="box")
    srdfTable = parse_srdf ("srdf/table_140_70_73.srdf", packageName = "gerard_bauzil", prefix="table")
    for w in [ "grippers", "handles" ]:
        srdf[w] = dict()
        for d in [ srdfTalos, srdfBox, srdfTable ]:
            srdf[w].update (d[w])

    supervisor = Supervisor (robot, hpTasks = hpTasks(robot))
    factory = Factory(supervisor)
    factory.parameters["period"] = 0.001 # TODO soon: robot.getTimeStep()
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = False
    factory.parameters["useMeasurementOfObjectsPose"] = False
    # factory.parameters["addTimerToSotControl"] = True
    factory.setGrippers (grippers)
    factory.setObjects (objects, handlesPerObjects, contactPerObjects)
    factory.environmentContacts (["table/top",])

    #factory.setRules (rules)
    factory.setupFrames (srdf["grippers"], srdf["handles"], robot,
      )
      #disabledGrippers=grippers)
      #disabledGrippers=["table/pose",])
    for gripper in grippers:
      for handle in handlesPerObjects[0]:
        factory.addAffordance (
          Affordance (gripper, handle,
            openControlType="torque", closeControlType="torque",
            refs = { "angle_open": (0,), "angle_close": (-0.5,), "torque": (-0.07,) },
            controlParams = { "torque_num": ( 5000., 1000.),
                "torque_denom": (0.01,) },
            simuParams = { "refPos": (-0.2,) }))
      factory.addAffordance (
        Affordance (gripper, None,
          openControlType="position", closeControlType="position",
          refs = { "angle_open": (0,), "angle_close": (-0.5,), "torque": (-0.05,) },
          simuParams = { "refPos": (-0.2,) }))
    factory.generate ()

    supervisor.makeInitialSot ()

    # starting_motion: From half_sitting to position where gaze and COM constraints are satisfied.
    sot_loop =supervisor.sots['Loop | f']
    supervisor.addSot("starting_motion", sot_loop, sot_loop.control)
    supervisor.addSot("loop_ss", sot_loop, sot_loop.control)
    supervisor.addSot("go_to_starting_state", sot_loop, sot_loop.control)
    return supervisor

# Use service /agimus/sot/set_base_pose to set initial config
# robot.device.set (tuple([-0.6,-0.2,] + list(robot.device.state.value[2:])))
supervisor = makeSupervisorWithFactory (robot)

supervisor.plugTopicsToRos()
supervisor.setupEvents ()
supervisor.plugSot("")
