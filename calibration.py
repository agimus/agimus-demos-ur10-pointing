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
from pinocchio import SE3, exp3, RobotWrapper, forwardKinematics, Quaternion
from agimus_demos.calibration import HandEyeCalibration as Parent
from tools_hpp import PathGenerator, RosInterface
from hpp import Transform
from hpp.corbaserver.manipulation import Constraints, SecurityMargins

## Hand-eye calibration
#
#  This class provides methods to generate configurations  where the robot
#  camera looks at the center of a Chessboard in order to perform hand eye
#  calibration.
#
#  To play the paths on the robot and collect data, see play_path.py in this
#  directory.
class Calibration(Parent):
    chessboardCenter = (0, 0, 1.41)
    # to aim at the center of the plaque with tubes, use
    # chessboardCenter = (0, 0, 1.17)

    # Number of configurations to generate for calibration
    nbConfigs = 10
    def __init__(self, ps, graph, factory):
        super(Calibration, self).__init__(ps, graph)
        self.factory = factory
        Parent.transition = 'go-look-at-cb'

    def generateValidConfigs(self, q0, n, m, M):
        robot = self.ps.robot
        graph = self.graph
        result = list()
        i = 0
        while i < n:
            q = robot.shootRandomConfig()
            res, q1, err = graph.generateTargetConfig('go-look-at-cb', q0, q)
            if not res: continue
            # Check that coordinate of chessboard center along z is between
            # m and M.
            robot.setCurrentConfig(q1)
            wMc = Transform(robot.getLinkPosition\
                            ('ur10e/camera_color_optical_frame'))
            wMp = Transform(robot.getLinkPosition('part/base_link'))
            # Position of chessboard center in world frame
            c = wMp.transform(np.array(self.chessboardCenter))
            # Position of chessboard center in camera frame
            c1 = wMc.inverse().transform(c)
            if not (m < c1[2] and c1[2] < M): continue
            res, msg = robot.isConfigValid(q1)
            if res:
                result.append(q1)
                i += 1
        return result

    def addStateToConstraintGraph(self):
        if 'look-at-cb' in self.graph.nodes.keys():
            return
        ps = self.ps; graph = self.graph
        ps.createPositionConstraint\
            ('look-at-cb', 'ur10e/camera_color_optical_frame', 'part/base_link',
             (0, 0, 0), self.chessboardCenter,
             (True, True, False))

        ps.createTransformationConstraint('placement/complement', '',
                                          'part/root_joint',
                                          [0,0,0,0, 0, 0, 1],
                                          [True, True, True, True, True, True,])
        ps.setConstantRightHandSide('placement/complement', False)

        graph.createNode(['look-at-cb'])
        graph.createEdge('free', 'look-at-cb', 'go-look-at-cb', 1,
                         'free')
        graph.createEdge('look-at-cb', 'free', 'stop-looking-at-cb', 1,
                         'free')

        graph.addConstraints(node='look-at-cb',
            constraints = Constraints(numConstraints=['look-at-cb']))
        graph.addConstraints(edge='go-look-at-cb',
            constraints = Constraints(numConstraints=['placement/complement']))
        graph.addConstraints(edge='stop-looking-at-cb',
            constraints = Constraints(numConstraints=['placement/complement']))

        self.factory.generate()
        sm = SecurityMargins(ps, self.factory, ["ur10e", "part"])
        sm.setSecurityMarginBetween("ur10e", "ur10e", 0.02)
        sm.setSecurityMarginBetween("ur10e", "part", 0.1)
        sm.defaultMargin = 0.01
        sm.apply()
        graph.initialize()

    def generateConfigurationsAndPaths(self, q0, filename = None):
        ri = RosInterface(self.ps.robot)
        q_init = ri.getCurrentConfig(q0)
        if filename:
            configs = self.readConfigsInFile(filename)
        else:
            configs = self.generateValidConfigs(q0, self.nbConfigs, .3, .5)
            self.writeConfigsInFile("/tmp/ur10-configs.csv", configs)
        configs = [q_init] + configs
        self.buildRoadmap(configs)
        configs = self.orderConfigurations(configs)[:self.nbConfigs+1]
        self.visitConfigurations(configs)

# Generate a csv file with the following data for each configuration:
#  x, y, z, phix, phiy, phiz,
#  ur10e/shoulder_pan_joint, ur10e/shoulder_lift_joint, ur10e/elbow_joint,
#  ur10e/wrist_1_joint, ur10e/wrist_2_joint, ur10e/wrist_3_joint
#  corresponding to the pose of the camera in the world frame (orientation in
#  roll, pitch, yaw), and the joints angles of the robot.
#
#  maxIndex is the maximal index of the input files:
#    configuration_{i}, i<=maxIndex,
#    pose_cPo_{i}.yaml, i<=maxIndex.
def generateDataForFigaroh(robot, input_directory, output_file, maxIndex):
    # create a pinocchio model from the Robot.urdfString
    with open('/tmp/robot.urdf', 'w') as f:
        f.write(robot.urdfString)
    pinRob = RobotWrapper()
    pinRob.initFromURDF('/tmp/robot.urdf')
    # get camera frame id
    camera_frame_id = pinRob.model.getFrameId('camera_color_optical_frame')
    with open(output_file, 'w') as f:
        # write header line in output file
        f.write('x1,y1,z1,phix1,phiy1,phiz1,shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint\n')
        count = 1
        while count <= maxIndex:
            poseFile = os.path.join(input_directory, f'pose_cPo_{count}.yaml')
            configFile = os.path.join(input_directory, f'configuration_{count}')
            try:
                f1 = open(poseFile, 'r')
                d = yaml.safe_load(f1)
                f1.close()
                f1 = open(configFile, 'r')
                config = f1.readline()
                f1.close()
                cMo_tuple = list(zip(*d['data']))[0]
                trans = np.array(cMo_tuple[:3])
                rot = exp3(np.array(cMo_tuple[3:]))
                cMo = SE3(translation = trans, rotation = rot)
                # The first time, compute measured pose of chessboard to express
                # camera pose in world frame
                if count == 1:
                    q = np.array(list(map(float, config.split(','))))
                    forwardKinematics(pinRob.model, pinRob.data, q)
                    pinRob.framesForwardKinematics(q)
                    wMc = pinRob.data.oMf[camera_frame_id]
                    wMo = wMc * cMo
                    print(f'wMo = {wMo}')
                oMc = cMo.inverse()
                wMc = wMo * oMc
                line = ""
                # write camera pose
                for i in range(3):
                    line += f'{wMc.translation[i]},'
                rpy = pinocchio.rpy.matrixToRpy(wMc.rotation)
                for i in range(3):
                    line += f'{rpy[i]},'
                # write configuration
                line += config
                f.write(line)
            except FileNotFoundError as exc:
                print(f'{poseFile} does not exist')
            count+=1

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
    configurations, wMcs = readDataFromFigaroh(figaroh_csv_file, q_init)
    for q, wMc in zip(configurations, wMcs):
        displayPose(v, 'robot/chessboard', wMc)
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
