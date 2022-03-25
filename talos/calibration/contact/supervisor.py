# Copyright 2021 CNRS - Airbus SAS
# Author: Florent Lamiraux, Joseph Mirabel
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
import os
import numpy as np
from dynamic_graph import plug
from hpp.corbaserver.manipulation import Rule
from dynamic_graph.sot.core.task import Task
from dynamic_graph.sot.core.feature_pose import FeaturePose
from agimus_sot.sot import ContactAdmittance

def hpTasks(sotrobot):
    from agimus_sot.task import COM, Foot

    com = COM("talos", sotrobot)
    lf = Foot("talos/leg_left_6_joint", sotrobot)
    rf = Foot("talos/leg_right_6_joint", sotrobot)
    return com + lf + rf

def addContactDetection(supervisor, factory):
    for name in ["wrist_left_ft_link", "wrist_right_ft_link"]:
        if not robot.dynamic.hasSignal(name):
            robot.dynamic.createOpPoint(name, name)
    for ig, gripper in enumerate(factory.grippers):
        if gripper == 'talos/left_gripper':
            wrenchSignalName = 'forceLARM'
            ftLinkName = "wrist_left_ft_link"
        elif gripper == 'talos/right_gripper':
            wrenchSignalName = 'forceRARM'
            ftLinkName = "wrist_right_ft_link"
        else:
            raise RuntimeError('Unexpected gripper name: ' + gripper)
        for ih, handle in enumerate(factory.handles):
            edgeName = '{} > {} | f_12'.format(gripper, handle)
            name = 'pregrasp___{}___{}'.format(gripper, handle)
            task = Task(name + '_task')
            feature = FeaturePose(name + '_feature')
            ca = ContactAdmittance(name + '_contact')
            plug(feature.error, ca.errorIn)
            plug(feature.jacobian, ca.jacobianIn)
            plug(robot.device.signal(wrenchSignalName), ca.wrench)
            plug(robot.dynamic.signal('J'+ftLinkName), ca.ftJacobian)
            ca.threshold.value = 20
            ca.wrenchDes.value = np.array([0,0,30,0,0,0])
            ca.stiffness.value = 100 * np.identity(6)
            task.clear()
            task.add(ca.name)
            # Add task in transition that releases the contact
            edgeBack = '{} < {} | {}-{}_21'.format(gripper, handle, ig, ih)
            supervisor.sots[edgeBack] = supervisor.sots[edgeName]


def makeSupervisorWithFactory(robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.srdf_parser import parse_srdf, attach_all_to_link
    import pinocchio
    import rospy
    from rospkg import RosPack
    rospack = RosPack()

    if not hasattr(robot, "camera_frame"):
        robot.camera_frame = 'rgbd_rgb_optical_frame'

    srdf = {}
    # retrieve objects from ros param
    demoDict = rospy.get_param("/demo")
    robotDict = demoDict["robots"]
    if len(robotDict) != 1:
        raise RuntimeError("One and only one robot is supported for now.")
    objectDict = demoDict["objects"]
    objects = list(objectDict.keys())
    # parse robot and object srdf files
    srdfDict = dict()
    for r, data in robotDict.items():
        srdfDict[r] = parse_srdf(srdf = data["srdf"]["file"],
                                 packageName = data["srdf"]["package"],
                                 prefix=r)
    for o, data in objectDict.items():
        objectModel = pinocchio.buildModelFromUrdf\
                      (os.path.join(rospack.get_path(data["urdf"]["package"]),
                                    data["urdf"]["file"]))
        srdfDict[o] = parse_srdf(srdf = data["srdf"]["file"],
                                 packageName = data["srdf"]["package"],
                                 prefix=o)
        attach_all_to_link(objectModel, "base_link", srdfDict[o])

    grippers = list(demoDict["grippers"])
    handlesPerObjects = list()
    contactPerObjects = list()
    for o in objects:
        handlesPerObjects.append(sorted(list(srdfDict[o]["handles"].keys())))
        contactPerObjects.append(sorted(list(srdfDict[o]["contacts"].keys())))

    for w in ["grippers", "handles","contacts"]:
        srdf[w] = dict()
        for k, data in srdfDict.items():
            srdf[w].update(data[w])

    supervisor = Supervisor(robot, hpTasks=hpTasks(robot),
                            prefix=list(robotDict.keys())[0])
    factory = Factory(supervisor)
    factory.parameters["period"] = 0.001  # TODO soon: robot.getTimeStep()
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = False
    # factory.parameters["addTimerToSotControl"] = True
    factory.setGrippers(grippers)
    factory.setObjects(objects, handlesPerObjects, contactPerObjects)
    # forbid contact with 2 hands
    rules = [Rule(grippers=grippers, handles=(objects[0] + "/contact_*",
                                              objects[0] + "/contact_*"),
                  link=False),
             Rule(grippers=grippers, handles=(".*", ".*"), link=True)]
    factory.setRules(rules)
    factory.setupFrames(srdf["grippers"], srdf["handles"], robot)
    factory.generate()

    supervisor.makeInitialSot()

    # starting_motion: From half_sitting to position where gaze and COM constraints are satisfied.
    sot_loop = supervisor.sots["Loop | f"]
    supervisor.addSolver("starting_motion", sot_loop)
    supervisor.addSolver("loop_ss", sot_loop)
    supervisor.addSolver("go_to_starting_state", sot_loop)
    return factory, supervisor


# Use service /agimus/sot/set_base_pose to set initial config
factory, supervisor = makeSupervisorWithFactory(robot)
addContactDetection(supervisor, factory)

supervisor.plugTopicsToRos()
supervisor.plugSot("")
