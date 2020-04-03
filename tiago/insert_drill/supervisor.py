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
    # Two possible cases
    # - lock the base.
    # - make the base holonomic. This solution needs a bit more work
    #   as the HolonomicProjection creates a projection matrix which
    #   should be passed to all the SOT solvers.
    from agimus_sot.task import Task, Posture
    return Task()

def makeSupervisorWithFactory(robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.srdf_parser import parse_srdf

    grippers = "driller/drill_tip", "tiago/gripper",
    objects = "driller", "skin",
    handlesPerObjects = [ "driller/handle", ], [ "skin/hole", ],
    contactPerObjects = [], [],

    srdf = {}
    srdfTiago = parse_srdf("srdf/pal_hey5_gripper.srdf", packageName="tiago_data", prefix="tiago")
    srdfDriller = parse_srdf(
        "srdf/driller.srdf", packageName="gerard_bauzil", prefix="driller"
    )
    srdfQRDrill = parse_srdf(
        "srdf/qr_drill.srdf", packageName="gerard_bauzil", prefix="driller"
    )
    srdfSkin = parse_srdf(
        "srdf/aircraft_skin_with_marker.srdf", packageName="agimus_demos", prefix="skin"
    )
    for w in ["grippers", "handles","contacts"]:
        srdf[w] = dict()
        for d in [srdfTiago, srdfDriller, srdfQRDrill, srdfSkin]:
            srdf[w].update(d[w])

    supervisor = Supervisor(robot, hpTasks=hpTasks(robot))
    factory = Factory(supervisor)
    factory.parameters["period"] = 0.001  # TODO soon: robot.getTimeStep()
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = False
    # factory.parameters["addTimerToSotControl"] = True
    factory.setGrippers(grippers)
    factory.setObjects(objects, handlesPerObjects, contactPerObjects)

    from hpp.corbaserver.manipulation import Rule
    factory.setRules([
        # Tiago always hold the gripper.
        Rule([ "tiago/gripper", ], [ "driller/handle", ], True), Rule([ "tiago/gripper", ], [ ".*", ], False),
        # Allow to associate drill_tip with skin/hole only.
        Rule([ "driller/drill_tip", ], [ "driller/handle", ], False), Rule([ "driller/drill_tip", ], [ ".*", ], True), ])
    factory.setupFrames(srdf["grippers"], srdf["handles"], robot,
            disabledGrippers=["driller/drill_tip",])
    factory.gripperFrames["driller/drill_tip" ].hasVisualTag = True
    factory.handleFrames["skin/hole"].hasVisualTag = True
    factory.addAffordance(
        Affordance("tiago/gripper", "driller/handle",
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
    # factory.addAffordance(
    #     Affordance("driller/drill_tip", "skin/hole",
    #         openControlType="torque",
    #         closeControlType="position_torque",
    #         refs={
    #             "angle_open": (0,),
    #             "angle_close": (-0.5,),
    #             "torque": (-0.07,),
    #             },
    #         controlParams={
    #             "torque_num": (1.,),
    #             "torque_denom": (10.,1.),
    #             },
    #         simuParams={
    #             "M": 0.,
    #             "d": 5.,
    #             "k": 100.,
    #             "refPos": (-0.4,),
    #             },
    #         )
    factory.generate()

    supervisor.makeInitialSot()

    # starting_motion: From half_sitting to position where gaze and COM constraints are satisfied.
    # sot_loop = supervisor.sots["Loop | f"]
    # supervisor.addSolver("starting_motion", sot_loop)
    # supervisor.addSolver("loop_ss", sot_loop)
    # supervisor.addSolver("go_to_starting_state", sot_loop)
    return supervisor


# Use service /agimus/sot/set_base_pose to set initial config
supervisor = makeSupervisorWithFactory(robot)

supervisor.plugTopicsToRos()
supervisor.plugSot("")

assert robot.device.control.isPlugged()
print(robot.device.control.name)
