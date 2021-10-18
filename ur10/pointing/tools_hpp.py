# BSD 2-Clause License

# Copyright (c) 2021, Florent Lamiraux
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from pinocchio import XYZQUATToSE3, SE3ToXYZQUAT

tool_gripper = 'ur10e/gripper'
rosNodeStarted = False

def initRosNode():
    if not rosNodeStarted:
        rospy.init_node("hpp", disable_signals=True)

class ConfigGenerator(object):
    def __init__(self, graph):
        self.robot = graph.robot
        self.graph = graph

    def generateValidConfigForHandle(self, handle, qinit, qguesses = [],
                                     NrandomConfig=10):
        edge = tool_gripper + " > " + handle
        ok = False
        from itertools import chain
        def project_and_validate(e, qrhs, q):
            res, qres, err = self.graph.generateTargetConfig (e, qrhs, q)
            return res and self.robot.configIsValid(qres), qres
        qpg, qg = None, None
        for qrand in chain(qguesses, (self.robot.shootRandomConfig()
                                      for _ in range(NrandomConfig))):
            res, qpg = project_and_validate (edge+" | f_01", qinit, qrand)
            if res:
                ok, qg = project_and_validate(edge+" | f_12", qpg, qpg)
                if ok: break
        return ok, qpg, qg

    def generateValidConfig(self, constraint, qguesses = [], NrandomConfig=10):
        from itertools import chain
        for qrand in chain(qguesses, (self.robot.shootRandomConfig()
                                      for _ in range(NrandomConfig))):
            res, qres = constraint.apply (qrand)
            if res and self.robot.configIsValid(qres):
                return True, qres
        return False, None

class RosInterface(object):
    nodeId = 0
    def __init__(self, robot):
        self.robot = robot
        self.robotPrefix = robot.robotNames[0] + "/"
        initRosNode()

    def getCurrentConfig(self, q0, timeout=5.):
        from sensor_msgs.msg import JointState
        q = q0[:]
        # Acquire robot state
        msg = rospy.wait_for_message("/joint_states", JointState)
        for ni, qi in zip(msg.name, msg.position):
            jni = self.robotPrefix + ni
            if self.robot.getJointConfigSize(jni) != 1:
                continue
            try:
                rk = self.robot.rankInConfiguration[jni]
            except KeyError:
                continue
            assert self.robot.getJointConfigSize(jni) == 1
            q[rk] = qi

        return q

    def getObjectPose(self, q0, timeout=5.):
        # the object should be placed wrt to the robot, as this is what the
        # sensor tells us.
        # Get pose of object wrt to the camera using TF
        import tf2_ros, rospy

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        cameraFrame = self.robot.opticalFrame
        qres = q0[:]

        for obj in self.robot.robotNames[1:]:
            objectFrame = obj + '/base_link_measured'
            wMc = XYZQUATToSE3(self.robot.hppcorba.robot.getJointsPosition\
                               (q0, [self.robotPrefix + cameraFrame])[0])
            try:
                _cMo = tfBuffer.lookup_transform(cameraFrame, objectFrame,
                        rospy.Time(), rospy.Duration(timeout))
                _cMo = _cMo.transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                print('could not get TF transform : ', e)
                raise RuntimeError(str(e))
            cMo = XYZQUATToSE3([_cMo.translation.x, _cMo.translation.y,
                                _cMo.translation.z, _cMo.rotation.x,
                                _cMo.rotation.y, _cMo.rotation.z,
                                _cMo.rotation.w])
            rk = self.robot.rankInConfiguration[obj + '/root_joint']
            assert self.robot.getJointConfigSize(obj + '/root_joint') == 7
            qres[rk:rk+7] = SE3ToXYZQUAT (wMc * cMo)

        return qres
