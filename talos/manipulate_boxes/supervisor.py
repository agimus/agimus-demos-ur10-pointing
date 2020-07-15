# Copyright 2018, 2019, 2020 CNRS - Airbus SAS
# Author: Florent Lamiraux, Joseph Mirabel, and Alexis Nicolin
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Theses variables are defined:
# - robot, a SoT device
# - simulateTorqueFeedbackForEndEffector, a boolean


def hpTasks(sotrobot):
    from agimus_sot.task import COM, Foot

    com = COM("talos", sotrobot)
    lf = Foot("talos/leg_left_6_joint", sotrobot)
    rf = Foot("talos/leg_right_6_joint", sotrobot)
    return com + lf + rf


def makeSupervisorWithFactory(robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.srdf_parser import parse_srdf

    if not hasattr(robot, "camera_frame"):
        robot.camera_frame = "rgbd_rgb_optical_frame"

    grippers = ["talos/left_gripper", "talos/right_gripper"]
    objects = ["box"]
    handlesPerObjects = [["box/handle" + str(i) for i in range(1, 5)]]
    contactPerObjects = [["box/front_surface", "box/rear_surface"]]

    srdf = {}
    srdfTalos = parse_srdf("srdf/talos.srdf", packageName="talos_data", prefix="talos")
    srdfBox = parse_srdf(
        "srdf/plank_of_wood1.srdf", packageName="gerard_bauzil", prefix="box"
    )
    srdfTable = parse_srdf(
        "srdf/rolling_table.srdf", packageName="gerard_bauzil", prefix="table"
    )
    for w in ["grippers", "handles","contacts"]:
        srdf[w] = dict()
        for d in [srdfTalos, srdfBox, srdfTable]:
            srdf[w].update(d[w])

    supervisor = Supervisor(robot, hpTasks=hpTasks(robot))
    factory = Factory(supervisor)
    factory.parameters["period"] = 0.001  # TODO soon: robot.getTimeStep()
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = False
    factory.parameters["addTracerToVisualServoing"] = True
    # factory.parameters["addTimerToSotControl"] = True
    factory.setGrippers(grippers)
    factory.setObjects(objects, handlesPerObjects, contactPerObjects)
    factory.environmentContacts(["table/top"])

    # factory.setRules (rules)
    factory.setupFrames(srdf["grippers"], srdf["handles"], robot)
    factory.setupContactFrames(srdf["contacts"])
    # disabledGrippers=grippers)
    # disabledGrippers=["table/pose",])
    factory.gripperFrames["talos/left_gripper" ].hasVisualTag = True
    factory.gripperFrames["talos/right_gripper"].hasVisualTag = True
    for i in range(1,5):
        factory.handleFrames["box/handle"+str(i)].hasVisualTag = True
    factory.contactFrames["table/top"].hasVisualTag = True
    factory.contactFrames["box/front_surface"].hasVisualTag = True
    for gripper in grippers:
        for handle in handlesPerObjects[0]:
            factory.addAffordance(
                Affordance(
                    gripper,
                    handle,
                    openControlType="torque",
                    closeControlType="position_torque",
                    refs={
                        "angle_open": (0,),
                        "angle_close": (-0.5,),
                        "torque": (-0.07,),
                    },
                    controlParams={
                        "torque_num": (1.,),
                        "torque_denom": (10.,1.),
                    },
                    simuParams={
                        "M": 0.,
                        "d": 5.,
                        "k": 100.,
                        "refPos": (-0.4,),
                        },
                )
            )
        factory.addAffordance(
            Affordance(
                gripper,
                None,
                openControlType="position",
                closeControlType="position",
                refs={"angle_open": (0,), "angle_close": (-0.5,), "torque": (-0.05,)},
                simuParams={"refPos": (-0.2,)},
            )
        )
    factory.generate()

    supervisor.makeInitialSot()

    # starting_motion: From half_sitting to position where gaze and COM constraints are satisfied.
    sot_loop = supervisor.sots["Loop | f"]
    supervisor.addSolver("starting_motion", sot_loop)
    supervisor.addSolver("loop_ss", sot_loop)
    supervisor.addSolver("go_to_starting_state", sot_loop)
    return supervisor


# Use service /agimus/sot/set_base_pose to set initial config
# robot.device.set (tuple([-0.6,-0.2,] + list(robot.device.state.value[2:])))
supervisor = makeSupervisorWithFactory(robot)

supervisor.plugTopicsToRos()
supervisor.plugSot("")
