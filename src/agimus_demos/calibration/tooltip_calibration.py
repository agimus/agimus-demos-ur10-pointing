# Copyright 2022 CNRS - Airbus SAS
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

from csv import reader
import numpy as np
from pinocchio import integrate, JointModelFreeFlyer, Model, Quaternion, \
    SE3
from pinocchio.rpy import matrixToRpy

def cross(v):
    return np.cross(v, np.identity(v.shape[0]) * -1)

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

##
#  Qt Widget to perform hand eye calibration of the UR10 robot
#
# This class provides methods to
#
#   - align the tooltip with a hole via an offset of the gripper
#     position in the end effector frame,
#   - recording the offset together with the part pose in the camera frame,
#   - compute the camera pose in the end effector frame.

class TooltipCalibration(object):

    ##
    #  \param cMe estimated pose of end effector in camera frame
    def __init__(self, cMe = None, oh = None):
        self.cMe = cMe
        self.oh = oh
        if self.cMe is None:
            self.cMe = SE3(Quaternion(np.array([0.339, 0.342, 0.617, 0.623])),
                           np.array([0.072, -0.001, -0.094]))
        self.et = np.array([0,0,0.25])
        if self.oh is None:
            self.oh = np.array([1.07500000e-01, 0, 1.32])
        self.measurements = list()

    def readData(self, filename):
        with open(filename, 'r') as f:
            r = reader(f)
            for i, line in enumerate(r):
                self.measurements.append(self.parseLine(line, i))
        # allocate vector and Jacobian matrix
        self.value = np.zeros(3*len(self.measurements))
        self.jacobian = np.zeros(6*3*len(self.measurements))
        self.jacobian.resize(3*len(self.measurements), 6)

    def solve(self):
        integration = SE3Integrator()
        # Compute pose of camera in end-effector frame
        for i in range(20):
            self.computeValueAndJacobian()
            print(f'squared error = {sum(self.value*self.value)}')
            nu = -np.matmul(np.linalg.pinv(self.jacobian),self.value)
            self.cMe = integration(self.cMe, nu)
        # Compute position of tooltip in camera frame
        if len(self.measurements) == 0: return
        s = 0
        for m in self.measurements:
            s += m['cMo'].act(self.oh)
        ch = s/len(self.measurements)
        # There is an intermediate frame called ref_camera_link
        # between ur10e_d435_mount_link and the camera optical frame
        # camera_color_optical_frame. Let us denote by i for this frame
        # eMc = eMi * iMc
        # iMc is provided by the camera manufacturer. We need to compute
        # eMi since this is the value reported in the urdf file.
        #
        # eMi = eMc * iMc.inverse()
        iMc = SE3(Quaternion(np.array([-.5,.5,-.5,.5])),
                             np.array([0.011, 0.033, 0.013]))
        eMc = self.cMe.inverse()
        self.eMi = eMc * iMc.inverse()
        self.eh = eMc.act(ch)
        offset = self.eh - self.et
        print(f"""   <xacro:arg name="tip_offset_x" default="{offset[0]}"/>
   <xacro:arg name="tip_offset_y" default="{offset[1]}"/>
   <xacro:arg name="tip_offset_z" default="{offset[2]}"/>
        """)
        rpy = matrixToRpy(self.eMi.rotation)
        xyz = self.eMi.translation
        print(f"""    <joint name="ref_camera_joint" type="fixed">
     <parent link="ur10e_d435_mount_link" />
     <child link = "ref_camera_link" />
     <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}"
	     rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>
   </joint>
        """)

    def computeValueAndJacobian(self):
        for i, m in enumerate(self.measurements):
            self.value[3*i:3*i+3] = self.cMe.act(self.et + m['offsets']) - \
                m['cMo'].act(self.oh)
            self.jacobian[3*i+0:3*i+3,0:3] = self.cMe.rotation
            self.jacobian[3*i+0:3*i+3,3:6] = -np.matmul(self.cMe.rotation,
                cross(self.et + m['offsets']))

    def parseLine(self, line, i):
        # Check that line starts with joint_states
        if line[0] != 'offsets':
            raise RuntimeError('line {} does not start by keyword "offsets"'
                               .format(i))
        measurement = dict()
        try:
            measurement['offsets'] = np.array(list(map(float, line [1:4])))
        except ValueError as exc:
            raise SyntaxError(f'line {i+1}, tag "offsets": could not convert' +\
                              f' list {line [1:4]} to array')
        if line[4] != 'cMo':
            raise SyntaxError(f'line {i+1}, expected tag "cMo": got {line[4]}')
        try:
            v = list(map(float, line [5:12]))
            p = Quaternion(x=v[3], y=v[4], z=v[5], w=v[6])
            t = np.array(v[0:3]).reshape(3,1)
            measurement ['cMo'] = SE3(p,t)
        except ValueError as exc:
            raise SyntaxError(f'line {i+1}, tag "cMo": could not convert' +\
                              f' list {line [5:12]} to array')
        return measurement

