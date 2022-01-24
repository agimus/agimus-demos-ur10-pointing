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
from math import sqrt
import hpp_idl
from pinocchio import XYZQUATToSE3, SE3ToXYZQUAT
from agimus_demos import InStatePlanner
from hpp.corbaserver import wrap_delete
from hpp.corbaserver.manipulation import CorbaClient

def concatenatePaths(paths):
    if len(paths) == 0: return None
    p = paths[0].asVector()
    for q in paths[1:]:
        assert(p.end() == q.initial())
        p.appendPath(q)
    return p

tool_gripper = 'ur10e/gripper'
rosNodeStarted = False

def initRosNode():
    if not rosNodeStarted:
        rospy.init_node("hpp", disable_signals=True)

class PathGenerator(object):
    def __init__(self, ps, graph, ri=None, qref=None):
        self.ps = ps
        self.robot = ps.robot
        self.graph = graph
        self.ri = ri
        self.qref = qref # this configuration stores the default pose of the part
        # store corba object corresponding to constraint graph
        self.cgraph = ps.hppcorba.problem.getProblem().getConstraintGraph()
        # create Planner to solve path planning problems on manifolds
        self.inStatePlanner = InStatePlanner(ps, graph)

    def wd(self, o):
        return wrap_delete(o, self.ps.client.basic._tools)

    def generateValidConfigForHandle(self, handle, qinit, qguesses = [],
                                     NrandomConfig=10, isClogged=None, step=3):
        if isClogged is None:
            isClogged = lambda x : False
        edge = tool_gripper + " > " + handle
        ok = False
        from itertools import chain
        def project_and_validate(e, qrhs, q):
            res, qres, err = self.graph.generateTargetConfig (e, qrhs, q)
            return res and not isClogged(qres) and self.robot.configIsValid(qres), qres
        qpg, qg = None, None
        res = False
        for qrand in chain(qguesses, (self.robot.shootRandomConfig()
                                      for _ in range(NrandomConfig))):
            res1, qpg = project_and_validate (edge+" | f_01", qinit, qrand)
            if not res1: continue
            if step >= 2:
                res2, qg = project_and_validate(edge+" | f_12", qpg, qpg)
                if not res2:
                    continue
            else:
                res2 = True
            res = True; break
        return res, qpg, qg

    def generateValidConfig(self, constraint, qguesses = [], NrandomConfig=10):
        from itertools import chain
        for qrand in chain(qguesses, (self.robot.shootRandomConfig()
                                      for _ in range(NrandomConfig))):
            res, qres = constraint.apply (qrand)
            if res and self.robot.configIsValid(qres):
                return True, qres
        return False, None

    def checkQInit(self, qinit):
        if qinit is None:
            if self.ri is None or self.qref is None:
                raise ValueError('ri and qref must be defined')
            else:
                return self.ri.getCurrentConfig(self.qref)
        return qinit

    # Generate a path from an initial configuration to a grasp
    #
    # param handle: name of the handle,
    # param qinit: initial configuration of the system,
    # param NrandomConfig: number of trials to generate a pre-grasp
    #                      configuration,
    # param isClogged: function that checks whether the pregrasp and grasps
    # are clogged.
    # param step: final state of the motion:
    #              1 -> pregrasp,
    #              2 -> grasps,
    #              3 -> back to pregrap.
    # and going through
    # pregraps, grasp and pregrasp again for a given handle
    def generatePathForHandle(self, handle, qinit=None, NrandomConfig=10,
                              isClogged=None, step=3):
        qinit = self.checkQInit(qinit)
        if isClogged is None:
            isClogged = lambda x : False
        # generate configurations
        edge = tool_gripper + " > " + handle
        ok = False
        for nTrial in range(NrandomConfig):
            res, qpg, qg = self.generateValidConfigForHandle\
               (handle=handle, qinit=qinit, qguesses = [qinit],
                NrandomConfig=NrandomConfig, isClogged=isClogged, step=step)
            if not res:
                continue
            # build path
            # from qinit to pregrasp
            self.inStatePlanner.setEdge(edge + " | f_01")
            try:
                p1 = self.inStatePlanner.computePath(qinit, [qpg],
                                                     resetRoadmap = True)
            except hpp_idl.hpp.Error as exc:
                p1 = None
            if not p1: continue
            if step < 2:
                return p1
            # from pregrasp to grasp
            self.inStatePlanner.setEdge(edge + " | f_12")
            res, p2, msg = self.inStatePlanner.directPath(qpg, qg, True)
            if not res: p2 = None
            if not p2: continue
            # Return concatenation
            if step < 3:
                return concatenatePaths([p1, p2])
            # back to pregrasp
            p3 = self.wd(p2.reverse())
            return concatenatePaths([p1, p2, p3])
        raise RuntimeError('failed fo compute a path.')

    def goTo(self, qgoal, qinit=None):
        qinit = self.checkQInit(qinit)
        self.inStatePlanner.setEdge("Loop | f")
        p1 = self.inStatePlanner.computePath(qinit, [qgoal],
                                             resetRoadmap = True)
        return p1


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
                # renormalize quaternion
                x = _cMo.rotation.x
                y = _cMo.rotation.y
                z = _cMo.rotation.z
                w = _cMo.rotation.w
                n = sqrt(x*x+y*y+z*z+w*w)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                print('could not get TF transform : ', e)
                raise RuntimeError(str(e))
            cMo = XYZQUATToSE3([_cMo.translation.x, _cMo.translation.y,
                                _cMo.translation.z, _cMo.rotation.x/n,
                                _cMo.rotation.y/n, _cMo.rotation.z/n,
                                _cMo.rotation.w/n])
            rk = self.robot.rankInConfiguration[obj + '/root_joint']
            assert self.robot.getJointConfigSize(obj + '/root_joint') == 7
            qres[rk:rk+7] = SE3ToXYZQUAT (wMc * cMo)

        return qres
