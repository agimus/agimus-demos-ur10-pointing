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

from agimus_sot.react import TaskFactory
from agimus_sot.solver import Solver

Solver.maxControlSqrNorm = 20

# Action to be performed at start of pre-action of transition
# "ur10e/gripper > part/handle_{} | f_12"
class ObjectLocalization(object):
    timeout = 5
    def __init__(self, sotrobot, factory, gripper, handle):
        self.sotrobot = sotrobot
        self.objectLocalization = factory.tasks.getGrasp(gripper, handle)\
                                  ['pregrasp'].objectLocalization

    def __call__(self):
        ts = self.sotrobot.device.getTimeStep()
        to = int(self.timeout / self.sotrobot.device.getTimeStep())
        from time import sleep
        start_it = self.sotrobot.device.control.time
        while True:
            t = self.sotrobot.device.control.time
            if t > start_it + to:
                print("Failed to perform object localization")
                return False
            self.objectLocalization.trigger(t)
            if self.objectLocalization.done.value:
                print("Succesfully performed object localization")
                return True
            sleep(ts)


def makeSupervisorWithFactory(robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.srdf_parser import parse_srdf, attach_all_to_link
    import pinocchio
    import rospy
    from rospkg import RosPack
    rospack = RosPack()

    if not hasattr(robot, "camera_frame"):
        robot.camera_frame = "xtion_optical_frame"
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
                      (rospack.get_path(data["urdf"]["package"]) +
                       data["urdf"]["file"])
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

    supervisor = Supervisor(robot, prefix=list(robotDict.keys())[0])
    factory = Factory(supervisor)
    factory.tasks = TaskFactory(factory)
    factory.parameters["period"] = 0.01  # TODO soon: robot.getTimeStep()
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = False
    # factory.parameters["addTimerToSotControl"] = True
    factory.setGrippers(grippers)
    factory.setObjects(objects, handlesPerObjects, contactPerObjects)

    from hpp.corbaserver.manipulation import Rule
    factory.setupFrames(srdf["grippers"], srdf["handles"], robot)
    for k in handlesPerObjects[0]:
        factory.handleFrames[k].hasVisualTag = True
    factory.generate()

    supervisor.makeInitialSot()
    g = factory.grippers[0]
    for h in factory.handles:
        transitionName = '{} > {} | f_12'.format(g, h)
        goalName = '{} grasps {}'.format(g, h)
        # Add visual servoing in post actions of transtion 'g > h | f_12'
        # visual servoing is deactivated by default at this step to avoid
        # undesirable effects when grasping an object.
        supervisor.postActions[transitionName][goalName].sot = \
          supervisor.sots[transitionName].sot
        # Add a pre-action to the pre-action of transition 'g > h | f_12'
        # in order to perform object localization before starting the
        # motion.
        supervisor.preActions[transitionName].preActions.append\
            (ObjectLocalization(robot, factory, g, h))
    return factory, supervisor


# Use service /agimus/sot/set_base_pose to set initial config
factory, supervisor = makeSupervisorWithFactory(robot)

supervisor.plugTopicsToRos()
supervisor.plugSot("")
