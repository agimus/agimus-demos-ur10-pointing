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
