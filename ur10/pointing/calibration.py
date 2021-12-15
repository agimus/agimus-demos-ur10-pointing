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

from script_hpp import *
import pinocchio, hpp.rostools, hppfcl, numpy as np
from agimus_demos.calibration import Calibration
from tools_hpp import PathGenerator, RosInterface
from hpp import Transform

chessboardCenter = (0, 0, 1.41)

def generateValidConfigs(q, n, m, M):
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
        c = wMp.transform(np.array(chessboardCenter))
        # Position of chessboard center in camera frame
        c1 = wMc.inverse().transform(c)
        if not (m < c1[2] and c1[2] < M): continue
        res, msg = robot.isConfigValid(q1)
        if res:
            result.append(q1)
            i += 1
    return result

ps.createPositionConstraint\
    ('look-at-cb', 'ur10e/camera_color_optical_frame', 'part/base_link',
     (0, 0, 0), chessboardCenter,
     (True, True, False))

ps.createTransformationConstraint('placement/complement', '','part/root_joint',
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
                     constraints = Constraints(numConstraints=\
                                               ['placement/complement']))
graph.addConstraints(edge='stop-looking-at-cb',
                     constraints = Constraints(numConstraints=\
                                               ['placement/complement']))
factory.generate()
sm = SecurityMargins(ps, factory, ["ur10e", "part"])
sm.setSecurityMarginBetween("ur10e", "ur10e", 0.02)
sm.defaultMargin = 0.01
sm.apply()
graph.initialize()

ri = RosInterface(robot)
q_init = ri.getCurrentConfig(q0)
calibration = Calibration(ps, graph)
calibration.transition = 'go-look-at-cb'
configs = calibration.readConfigsInFile('./data/calib-configs.csv')
configs = [q_init] + configs
calibration.buildRoadmap(configs)
configs = calibration.orderConfigurations(configs)
calibration.visitConfigurations(configs)
