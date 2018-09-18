#/usr/bin/env python
from common_hpp import *
from hpp.corbaserver.manipulation.robot import CorbaClient
from hpp.corbaserver.manipulation import ProblemSolver
from hpp import Transform

import sys, numpy as np

clients = CorbaClient (postContextId = "")
clients.manipulation.problem.resetProblem()

robot, ps, vf = makeRobotProblemAndViewerFactory(clients)

q_init = robot.getCurrentConfig()

ps.addPartialCom ("talos", ["talos/root_joint"])
ps.addPartialCom ("talos_box", ["talos/root_joint", "box/root_joint"])

#ps.createStaticStabilityConstraints ("balance", half_sitting, "talos", ProblemSolver.FIXED_ON_THE_GROUND)
#foot_placement = [ "balance/pose-left-foot", "balance/pose-right-foot" ]
#foot_placement_complement = [ ]
ps.createStaticStabilityConstraints ("balance", half_sitting, "talos", ProblemSolver.SLIDING)
foot_placement = [ "balance/relative-orientation", "balance/relative-position", "balance/orientation-left-foot", "balance/position-left-foot" ]
foot_placement_complement = [ "balance/orientation-left-foot-complement", "balance/position-left-foot-complement" ]

robot.setCurrentConfig(half_sitting)
com_wf = np.array(ps.getPartialCom("talos"))
tf_la = Transform (robot.getJointPosition(robot.leftAnkle))
com_la = tf_la.inverse().transform(com_wf)

ps.createRelativeComConstraint ("com_talos_box", "talos_box", robot.leftAnkle, com_la.tolist(), (True, True, True))
ps.createRelativeComConstraint ("com_talos"    , "talos"    , robot.leftAnkle, com_la.tolist(), (True, True, True))

ps.createPositionConstraint ("gaze", "talos/rgbd_optical_joint", "box/root_joint", (0,0,0), (0,0,0), (True, True, False))

left_gripper_lock = []
right_gripper_lock = []
other_lock = ["talos/torso_1_joint"]
for n in robot.jointNames:
    s = robot.getJointConfigSize(n)
    r = robot.rankInConfiguration[n]
    if n.startswith ("talos/gripper_right"):
        ps.createLockedJoint(n, n, half_sitting[r:r+s])
        ps.setConstantRightHandSide(n, True)
        right_gripper_lock.append(n)
    elif n.startswith ("talos/gripper_left"):
        ps.createLockedJoint(n, n, half_sitting[r:r+s])
        ps.setConstantRightHandSide(n, True)
        left_gripper_lock.append(n)
    elif n in other_lock:
        ps.createLockedJoint(n, n, half_sitting[r:r+s])
        ps.setConstantRightHandSide(n, True)

graph = makeGraph (robot)

for nodename, nodeid in graph.nodes.iteritems():
    graph.addConstraints (node=nodename,
        lockDof = left_gripper_lock + right_gripper_lock + other_lock,
        numConstraints = [ "com_talos_box", "gaze"])

graph.createNode("starting_state")
graph.createEdge("starting_state", "free", "starting_motion", isInNode="starting_state")
graph.addConstraints(node="starting_state",
	numConstraints = ["place_box", ] )
graph.addConstraints(edge="starting_motion",
	numConstraints = ["place_box/complement", ] )
for edgename, edgeid in graph.edges.iteritems():
    graph.addConstraints (edge=edgename,
        numConstraints = foot_placement_complement)

graph.setConstraints (graph=True,
        numConstraints = foot_placement)
ps.selectPathValidation ("Progressive", 0.05)
graph.initialize()

res, hs_proj, err = graph.applyNodeConstraints("starting_state", half_sitting)
# res, q_init, err = graph.applyNodeConstraints("free", half_sitting)
res, q_init, err = graph.generateTargetConfig("Loop | f", hs_proj, hs_proj)
res, q_init, err = graph.applyNodeConstraints("free", q_init)
# res, q_goal, err = graph.generateTargetConfig("Loop | f", q_tmp, q_tmp)
res, q_goal, err = graph.generateTargetConfig("talos/left_gripper > box/handle1 | f_23", q_init, q_init)
# res, q_goal, err = graph.applyNodeConstraints("talos/right_gripper grasps box/bottom", half_sitting)
# print ps.directPath(q_init, q_init, True)
ps.setInitialConfig(q_init)
# ps.addGoalConfig(q_goal)
ps.setTargetState (graph.nodes["talos/left_gripper grasps box/handle1"])
ps.setParameter("SimpleTimeParameterization/safety", 0.5)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 2.)

ps.setParameter ("ConfigurationShooter/Gaussian/standardDeviation", 0.05)
ps.client.basic.problem.selectConfigurationShooter ("Gaussian")
robot.setCurrentConfig(q_init)
# ps.setRandomSeed(1)
# sys.exit(1)


res, pid, msg = ps.directPath (hs_proj, q_init, True)
if not res:
  print "Could not compute path from half_sitting to init"
else:
  ps.optimizePath (pid)

print ps.solve()
