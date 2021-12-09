from script_hpp import *

# ps.client.basic.problem.createTransformationConstraint2\
#     ('look-at-cb', 'ur10e/camera_color_optical_joint', 'part/root_joint',
#      (0, 0, 0, 0, 0, 0, 1), (0, 0, 1.41, 0, 0, 0, 1),
#      (True, True, False, False, False, False))

ps.createPositionConstraint\
    ('look-at-cb', 'ur10e/camera_color_optical_frame', 'part/base_link',
     (0, 0, 0), (0, 0, 1.41),
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
