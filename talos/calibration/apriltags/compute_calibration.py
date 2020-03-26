# Copyright 2020 CNRS - Airbus SAS
# Author: Florent Lamiraux
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

import os
import numpy as np, pinocchio
from numpy.linalg import norm, pinv
from csv import reader
from pinocchio import computeJointJacobians, forwardKinematics, \
    getJointJacobian, integrate, Jlog6, JointModelFreeFlyer, log6, Model, \
    neutral, Quaternion, ReferenceFrame, SE3
from pinocchio.robot_wrapper import RobotWrapper
pinocchio.switchToNumpyArray()

def parseLine(line, i):
    # check that line starts with joint_states
    if line [0] != 'joint_states':
        raise RuntimeError('line {} does not start by keyword "joint_states"'
                            .format(i))
    # look for keyword
    ilg = irg = None
    # make sure that one and only one of 'left_gripper' and 'right_gripper'
    # is specified in the current line.
    try:
        ilg = line.index('left_gripper')
    except ValueError as exc:
        pass
    try:
        irg = line.index('right_gripper')
    except ValueError as exc:
        pass
    if irg is None and ilg is None:
        raise SyntaxError \
           ('line {} contains neither "left_gripper" nor "right_gripper" tag.'
             .format(i+1))
    if not irg is None and not ilg is None:
        raise SyntaxError \
           ('line {} contains both "left_gripper" and "right_gripper" tags.'
             .format(i+1))
    measurement = dict()
    if ilg:
        try:
            measurement['joint_states'] = map(float, line [1:ilg])
        except ValueError as exc:
            raise SyntaxError\
           ('line {}, tag "joint_states": could not convert list {} to array'.
                               format(i+1, line [1:ilg]))
        try:
            v = map(float, line [ilg+1:])
            p = Quaternion(x=v[3], y=v[4], z=v[5], w=v[6])
            t = np.array(v[0:3]).reshape(3,1)
            measurement ['left_gripper'] = SE3(p,t)
        except ValueError as exc:
            raise SyntaxError\
           ('line {}, tag "left_gripper": could not convert list {} to array'.
                               format(i+1, line [ilg+1:]))
    if irg:
        try:
            measurement ['joint_states'] = map(float, line [1:irg])
        except ValueError as exc:
            raise SyntaxError\
           ('line {}, tag "joint_states": could not convert list {} to float'.
                               format(i+1, line [1:irg]))
        try:
            v = map(float, line [irg+1:])
            p = Quaternion(x=v[3], y=v[4], z=v[5], w=v[6])
            t = np.array(v[0:3]).reshape(3,1)
            measurement ['right_gripper'] =  SE3(p,t)
        except ValueError as exc:
            raise SyntaxError\
           ('line {}, tag "right_gripper": could not convert list {} to float'.
                               format(i+1, line [irg+1:]))
    return measurement

class Variable(object):
    """
    Optimization variable of the calibration problem. It contains
      - q_off: a vector of joint offsets,
      - hTc: a position of the camera in the head,
      - lwTls: a position of the left support in the left hand,
      - rwTrs: a position of the right support in the right hand.
    """
    def __init__(self, q_off, hTc, lwTls, rwTrs):
        self.q_off = q_off
        self.hTc = hTc
        self.lwTls = lwTls
        self.rwTrs = rwTrs

class SE3Integrator (object):
    """
    Integrate a velocity on SE3
    """
    def __init__(self):
        self.model = Model()
        self.model.addJoint(0, JointModelFreeFlyer(), SE3(), "SE3")

    def __call__(self, T, nu):
        """
        Integrate se3 velocity from SE3 element T
          - T: instance of SE3
          - nu: numpy array of dimension 6 (v,omega)
        """
        q0 = np.array(7*[0.])
        q0[0:3] = T.translation
        q0[3:7] = Quaternion(T.rotation).coeffs()
        q = integrate(self.model,q0,nu)
        return SE3(Quaternion(x=q[3], y=q[4], z=q[5], w=q[6]), q[0:3])

class ComputeCalibration(object):
    # List of joints between the wrists and the camera.
    # Other joints have no effect on the measurement. They are omitted
    joints = ['arm_left_1_joint','arm_left_2_joint','arm_left_3_joint',
              'arm_left_4_joint','arm_left_5_joint','arm_left_6_joint',
              'arm_left_7_joint',
              'arm_right_1_joint','arm_right_2_joint','arm_right_3_joint',
              'arm_right_4_joint','arm_right_5_joint','arm_right_6_joint',
              'arm_right_7_joint',
              'head_1_joint','head_2_joint']
    headJoint = 'head_2_joint'
    lwJoint = 'arm_left_7_joint'
    rwJoint = 'arm_right_7_joint'
    eps = 1
    def __init__(self, urdfFilename, variable):
        self.cols = len(self.joints) + 3*6
        # warning, there is no root joint
        # self.robot.model.joints[i].idx_v gives the index of the column of
        # the Jacobian corresponding to joint i
        # cc.robot.model.names[i] gives the name of joint i.
        self.robot = RobotWrapper.BuildFromURDF(urdfFilename)
        # record head and wrist indices in model
        self.headId = self.robot.model.getJointId(self.headJoint)
        self.lwId = self.robot.model.getJointId(self.lwJoint)
        self.rwId = self.robot.model.getJointId(self.rwJoint)
        assert(self.robot.model.names[self.headId] == self.headJoint)
        assert(self.robot.model.names[self.lwId] == self.lwJoint)
        assert(self.robot.model.names[self.rwId] == self.rwJoint)
        self.measurements = list()
        self.variable = variable
        self.integrate = SE3Integrator()
        # allocate constant matrices
        self.Jh = np.array(6*[len(self.joints)*[0.]])
        self.Jw = np.array(6*[len(self.joints)*[0.]])

    def readData(self, filename):
        jointNamesRead = False
        with open(filename, 'r') as f:
            r = reader(f)
            for i, line in enumerate(r):
                if not jointNamesRead:
                    if line [0] != 'joint_names':
                        raise SyntaxError \
                       ('expecting tag "joint_names" in first line, got "{}"'
                             .format(line [0]))
                    self.allJoints = line [1:]
                    jointNamesRead = True
                else:
                    self.measurements.append(parseLine(line, i))
        # Allocate value and Jacobian
        self.value = np.array(6*len(self.measurements)*[0.])
        self.value.shape =(6*len(self.measurements),1)
        self.jacobian = np.array(6*len(self.measurements)*self.cols*[0.])
        self.jacobian.shape =(6*len(self.measurements), self.cols)
        # Compute list of joint indices
        #  - measurementIndices indices of self.joints in configurations of
        #     measurements,
        #  - modelQIndices indices of self.joints in self.robot.model
        #     configurations
        #  - modelVIndices indices of self.joints in self.robot.model
        #     velocities.
        self.measurementIndices = list()
        self.modelQIndices = list()
        self.modelVIndices = list()
        model = self.robot.model
        for j in self.joints:
            i = model.getJointId(j)
            self.modelQIndices.append(model.joints[i].idx_q)
            self.modelVIndices.append(model.joints[i].idx_v)
            self.measurementIndices.append(self.allJoints.index(j))
        
    def computeValueAndJacobian(self):
        model = self.robot.model; data = self.robot.data
        nj = len(self.joints)
        for im, measurement in enumerate(self.measurements):
            #                            ^
            # compute configuration q  = q  + q
            #                        i    i    off
            qi = neutral(model)
            for i in range(nj):
                imeas = self.measurementIndices[i]
                imodel = self.modelQIndices[i]
                qi[imodel] = measurement['joint_states'][imeas] + \
                             self.variable.q_off[i,0]
            forwardKinematics(model, data, qi)
            computeJointJacobians(model, data)
            # position of the head
            Th = data.oMi[self.headId]
            hTc = self.variable.hTc
            if measurement.has_key('left_gripper'):
                meas_cTs = measurement['left_gripper']
                Tw = data.oMi[self.lwId]
                wTs = self.variable.lwTls
                Jw_full = getJointJacobian(model, data, self.lwId, \
                                           ReferenceFrame.LOCAL)
                left = True
            else:
                meas_cTs = measurement['right_gripper']
                Tw = data.oMi[self.rwId]
                wTs = self.variable.rwTrs
                Jw_full = getJointJacobian(model, data, self.rwId, \
                                           ReferenceFrame.LOCAL)
                left = False
            valueSE3 = meas_cTs.inverse()*hTc.inverse()*Th.inverse()*Tw*wTs
            value = log6(valueSE3)
            Jlog = Jlog6(valueSE3)
            self.value[6*im+0:6*im+6] = value.vector.reshape(6,1)
            # Compute Jacobian
            Xh = Th.toActionMatrix()
            Xw_inv = Tw.toActionMatrixInverse()
            wXs_inv = wTs.toActionMatrixInverse()
            hXc = hTc.toActionMatrix()
            sTc = wTs.inverse()*Tw.inverse()*Th*hTc;
            sXc = sTc.toActionMatrix()
            # Fill reduced Jacobian matrices
            Jh_full = getJointJacobian(model, data, self.headId, \
                                       ReferenceFrame.LOCAL)
            # columns relative to q_off
            for i,c in enumerate (self.modelVIndices):
                self.Jh[0:6,i:i+1] = Jh_full[0:6,c:c+1]
                self.Jw[0:6,i:i+1] = Jw_full[0:6,c:c+1]
            self.jacobian[6*im+0:6*im+6,0:nj] = \
                Jlog.dot(wXs_inv).dot(-Xw_inv.dot(Xh).dot(self.Jh) + self.Jw)
            # columns relative to hTc
            self.jacobian[6*im+0:6*im+6,nj:nj+6] = -Jlog.dot(sXc)
            # columns relative to lwTls and rwTrs
            if left:
                self.jacobian[6*im+0:6*im+6,nj+6: nj+12] = Jlog
                self.jacobian[6*im+0:6*im+6,nj+12:nj+18] = np.zeros((6,6))
            else:
                self.jacobian[6*im+0:6*im+6,nj+6: nj+12] = np.zeros((6,6))
                self.jacobian[6*im+0:6*im+6,nj+12:nj+18] = Jlog

    def solve(self):
        nj = len(self.joints)
        self.computeValueAndJacobian()
        dy = - self.eps * pinv(self.jacobian).dot(self.value)
        print ("||dy|| = {}".format(norm(dy)))
        self.variable.q_off += dy[0:nj]
        hnuc = dy[nj:nj+6]
        lwnuls = dy[nj+6:nj+12]
        rwnurs = dy[nj+12:nj+18]
        self.variable.hTc = self.integrate(self.variable.hTc, hnuc)
        self.variable.lwTls = self.integrate(self.variable.lwTls, lwnuls)
        self.variable.rwTrs = self.integrate(self.variable.rwTrs, rwnurs)

    def testJacobian(self):
        # Derivatives with respect to q_off
        nj = len(self.joints)
        dq = 1e-6
        self.computeValueAndJacobian()
        J = self.jacobian.copy()
        q_off0 = self.variable.q_off.copy()
        for i in range(nj):
            self.variable.q_off[i] = q_off0[i]-dq
            self.computeValueAndJacobian()
            v0 = self.value.copy()
            self.variable.q_off[i] = q_off0[i]+dq
            self.computeValueAndJacobian()
            v1 = self.value.copy()
            error = (v1 - v0)/(2*dq) - J[:,i:i+1]
            print ("||error|| = {}".format(norm(error)))
        self.variable.q_off = q_off0
        # Derivatives wrt hTc
        hTc0 = SE3(self.variable.hTc)
        for i in range(6):
            nu = np.zeros((6,1))
            nu[i] = -dq
            self.variable.hTc = self.integrate(hTc0, nu)
            self.computeValueAndJacobian()
            v0 = self.value.copy()
            nu[i] = dq
            self.variable.hTc = self.integrate(hTc0, nu)
            self.computeValueAndJacobian()
            v1 = self.value.copy()
            error = (v1 - v0)/(2*dq) - J[:,nj+i:nj+i+1]
            print ("||error|| = {}".format(norm(error)))
        self.variable.hTc = hTc0
        # Derivatives with respect to lwTls
        lwTls0 = SE3(self.variable.lwTls)
        for i in range(6):
            nu = np.zeros((6,1))
            nu[i] = -dq
            self.variable.lwTls = self.integrate(lwTls0, nu)
            self.computeValueAndJacobian()
            v0 = self.value.copy()
            nu[i] = dq
            self.variable.lwTls = self.integrate(lwTls0, nu)
            self.computeValueAndJacobian()
            v1 = self.value.copy()
            error = (v1 - v0)/(2*dq) - J[:,nj+6+i:nj+6+i+1]
            print ("||error|| = {}".format(norm(error)))
        self.variable.lwTls = lwTls0
        # Derivatives with respect to rwTrs
        rwTrs0 = SE3(self.variable.rwTrs)
        for i in range(6):
            nu = np.zeros((6,1))
            nu[i] = -dq
            self.variable.rwTrs = self.integrate(rwTrs0, nu)
            self.computeValueAndJacobian()
            v0 = self.value.copy()
            nu[i] = dq
            self.variable.rwTrs = self.integrate(rwTrs0, nu)
            self.computeValueAndJacobian()
            v1 = self.value.copy()
            error = (v1 - v0)/(2*dq) - J[:,nj+12+i:nj+12+i+1]
            print ("||error|| = {}".format(norm(error)))
        self.variable.rwTrs = rwTrs0

    def write_xacro(self):
        import itertools
        # The xacro file contains:
        # - the relative transform from head_2_link to rgbd_link, under camera_position_[xyz] and camera_orientation_[rpy] variables.
        # - the joint offsets, under variables <joint_name>_offset
        # Compute camera pose
        # hTc: head_2_joint (a) to rgbd_rgb_optical_frame (d)
        # desired: head_2_link (b) to rgbd_link (c)
        model = self.robot.model
        head_2_link = model.frames[model.getFrameId("head_2_link")]
        rgbd_rgb_optical_frame = model.frames[model.getFrameId("rgbd_rgb_optical_frame")]
        rgbd_link = model.frames[model.getFrameId("rgbd_link")]
        assert model.names[head_2_link.parent] == "head_2_joint"
        assert model.names[rgbd_rgb_optical_frame.parent] == "head_2_joint"
        assert model.names[rgbd_link.parent] == "head_2_joint"

        aMb = head_2_link.placement
        cMd = rgbd_link.placement.inverse() * rgbd_rgb_optical_frame.placement
        aMd = self.variable.hTc

        bMc = aMb.inverse() * aMd * cMd.inverse()
        xacro_property = """<xacro:property name="{}" value="{}" />"""
        for s, v in zip('xyz', bMc.translation):
            print(xacro_property.format("camera_position_"+s, v))
        for s, v in zip('rpy', pinocchio.rpy.matrixToRpy(bMc.rotation).ravel()):
            print(xacro_property.format("camera_orientation_"+s, v))

        for jn, v in zip(self.joints, self.variable.q_off.ravel()):
            print(xacro_property.format(jn+"_offset", -v))

if __name__ == '__main__':
    filename = os.getenv('DEVEL_HPP_DIR') + \
               '/install/share/talos_data/urdf/pyrene.urdf'

    nj = len(ComputeCalibration.joints)
    q_off = np.array(nj*[0.]).reshape(nj, 1)
    hTc = SE3(Quaternion(x=-0.500, y=0.500, z=-0.500, w=0.500),
              np.array([0.066, 0.013, 0.198]).reshape(3,1))
    lwTls = SE3(Quaternion(x=0, y=0, z=0, w=1),
                np.array([0.000, 0.000, -0.092]).reshape(3,1))
    rwTrs = SE3(Quaternion(x=0, y=0, z=1, w=0),
                np.array([0.000, 0.000, -0.092]).reshape(3,1))
    cc=ComputeCalibration(filename, Variable(q_off,hTc,lwTls,rwTrs))
    cc.readData('data/measurements-pyrene-20200212-2.csv')

    for i in range (20):
        cc.solve()
        print ("||error|| = {}".format(norm(cc.value)))
