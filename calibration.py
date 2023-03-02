# Copyright 2021 CNRS - Airbus SAS
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
import yaml
from csv import reader
import numpy as np
import eigenpy, pinocchio, hpp.rostools, hppfcl, numpy as np
from pinocchio import SE3, Quaternion
from agimus_demos.calibration import HandEyeCalibration as Calibration
from tools_hpp import PathGenerator, RosInterface
from hpp import Transform

def readDataFromFigaroh(filename, q0):
    with open(filename, 'r') as f:
        configurations = list()
        wMcs = list()
        r = reader(f)
        for i, line in enumerate(r):
            if i == 0: continue
            xyz = list(map(float, line[0:3])); rpy = list(map(float, line[3:6]))
            wMc = SE3(translation = np.array(xyz),
                      rotation = pinocchio.rpy.rpyToMatrix(np.array(rpy)))
            wMcs.append(wMc)
            # read robot configuration in csv file
            q = list(map(float, line[6:]))
            # append configuration of the plank
            q += q0[6:]
            configurations.append(q)
    return configurations, wMcs

def displayPose(v, nodeName, wMc):
    if not nodeName in v.client.gui.getNodeList():
        v.client.gui.addXYZaxis(nodeName, [1,1,1,1], 0.015, 0.1)
    pose = list(wMc.translation)
    pose += list(Quaternion(wMc.rotation).coeffs())
    v.client.gui.applyConfiguration(nodeName, pose)
    v.client.gui.refresh()

# Read configurations and cMo in files and display corresponding pose
# in gepetto-gui
def checkData(robot, v, figaroh_csv_file, q_init):
    view_from_camera = False
    configurations, wMcs = readDataFromFigaroh(figaroh_csv_file, q_init)
    for q, wMc in zip(configurations, wMcs):
        displayPose(v, 'robot/chessboard', wMc)
        if view_from_camera:
            ql = list(wMc.translation)
            ql += list((Quaternion(wMc.rotation)*
                        Quaternion(np.array([0,1,0,0]))).coeffs())
            v.client.gui.setCameraTransform('scene_hpp_', ql)
        v(q)
        print('Check pose of camera and press enter')
        input()

def computeCameraPose(mMe, eMc, eMc_measured):
    # we wish to compute a new position mMe_new of ref_camera_link in
    # ur10e_d435_mount_link in such a way that
    #  - eMc remains the same (we assume the camera is well calibrated),
    #  - mMc = mMe_new * eMc = mMe * eMc_measured
    # Thus
    #  mMe_new = mMe*eMc_measured*eMc.inverse()
    return mMe*eMc_measured*eMc.inverse()

# # Current position of ref_camera_link in ur10e_d435_mount_link
# mMe = pinocchio.SE3(translation=np.array([0.000, 0.117, -0.010]),
#                     quat = eigenpy.Quaternion(np.array(
#                         [-0.341, -0.340, -0.619, 0.620])))

# # Current position of camera_color_optical_frame in ref_camera_link
# eMc = pinocchio.SE3(translation=np.array([0.011, 0.033, 0.013]),
#                     quat = eigenpy.Quaternion(np.array(
#                         [-0.500, 0.500, -0.500, 0.500])))

# # Measured position of camera_optical_frame in ref_camera_link from calibration
# eMc_measured = pinocchio.SE3(translation=np.array(
#     [0.009813399648, 0.03326932627, 0.01378242934]),
#                              quat = eigenpy.Quaternion(np.array(
#     [-0.498901464, 0.5012362124, -0.4974698347, 0.5023776988])))

# #new position mMe_new of ref_camera_link in ur10e_d435_mount_link
# mMe_new = computeCameraPose(mMe, eMc, eMc_measured)
# xyz = mMe_new.translation
# rpy = pinocchio.rpy.matrixToRpy(mMe_new.rotation)
