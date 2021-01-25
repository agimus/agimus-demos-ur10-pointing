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
    # - add task that tracks the base position obtained via ROS.
    from agimus_sot.task import Task, Posture
    task = Task()
    from dynamic_graph.sot.core.matrix_constant import MatrixConstant
    import numpy as np
    N = sotrobot.dynamic.getDimension()
    projection = MatrixConstant("base_projection")
    projection.set(np.vstack((np.zeros((6, N-6)), np.identity(N-6))))
    task.projector = projection.sout
    return task

def makeSupervisorWithFactory(robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.srdf_parser import parse_srdf, attach_all_to_link
    import pinocchio

    if not hasattr(robot, "camera_frame"):
        robot.camera_frame = "xtion_optical_frame"

    srdf = {}
    srdfTiago = parse_srdf("srdf/pal_hey5_gripper.srdf", packageName="tiago_data", prefix="tiago")
    srdfPart = parse_srdf("srdf/cylinder.srdf", packageName="agimus_demos", prefix="part")

    grippers = "tiago/gripper",
    objects = "part",
    handlesPerObjects = sorted(srdfPart["handles"].keys()),
    contactPerObjects = [],

    for w in ["grippers", "handles","contacts"]:
        srdf[w] = dict()
        for d in [srdfTiago, srdfPart]:
            srdf[w].update(d[w])

    supervisor = Supervisor(robot, hpTasks=hpTasks(robot))
    factory = Factory(supervisor)
    factory.parameters["period"] = 0.01  # TODO soon: robot.getTimeStep()
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = False
    # factory.parameters["addTimerToSotControl"] = True
    factory.setGrippers(grippers)
    factory.setObjects(objects, handlesPerObjects, contactPerObjects)

    from hpp.corbaserver.manipulation import Rule
    factory.setRules([ Rule([ "tiago/gripper", ], [ ".*", ], True), ])
    factory.setupFrames(srdf["grippers"], srdf["handles"], robot)
    for k in handlesPerObjects[0]:
        factory.handleFrames[k].hasVisualTag = True
    #factory.addAffordance(
    #    Affordance("tiago/gripper", "driller/handle",
    #        openControlType="position",
    #        closeControlType="position",
    #        refs={
    #            "angle_open": (0.,0.,0.),
    #            "angle_close": (5.3,5.72,8.0),  #"angle_close": (6.2,6.7,9.1),
    #            },
    #        )
    #    )
    factory.generate()

    supervisor.makeInitialSot()
    return supervisor


# Use service /agimus/sot/set_base_pose to set initial config
supervisor = makeSupervisorWithFactory(robot)

supervisor.plugTopicsToRos()
supervisor.plugSot("")
