# /usr/bin/env python
import numpy as np
from hpp import Transform
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.corbaserver.manipulation.robot import CorbaClient

from common_hpp import *

clients = CorbaClient(postContextId="_estimation")
clients.manipulation.problem.resetProblem()

robot, ps, vf = makeRobotProblemAndViewerFactory(clients)
ps.setErrorThreshold(1e-2)

q_init = robot.getCurrentConfig()

ps.addPartialCom("talos", ["talos/root_joint"])
ps.addPartialCom("talos_box", ["talos/root_joint", "box/root_joint"])

# ps.createStaticStabilityConstraints ("balance", half_sitting, "talos", ProblemSolver.FIXED_ON_THE_GROUND)
# foot_placement = [ "balance/pose-left-foot", "balance/pose-right-foot" ]
# foot_placement_complement = [ ]
ps.createStaticStabilityConstraints(
    "balance", half_sitting, "talos", ProblemSolver.SLIDING
)
foot_placement = [
    "balance/relative-orientation",
    "balance/relative-position",
    "balance/orientation-left-foot",
    "balance/position-left-foot",
]
foot_placement_complement = [
    "balance/orientation-left-foot-complement",
    "balance/position-left-foot-complement",
]

robot.setCurrentConfig(half_sitting)
com_wf = np.array(ps.getPartialCom("talos"))
tf_la = Transform(robot.getJointPosition(robot.leftAnkle))
com_la = tf_la.inverse().transform(com_wf)

ps.createRelativeComConstraint(
    "com_talos_box", "talos_box", robot.leftAnkle, com_la.tolist(), (True, True, True)
)
ps.createRelativeComConstraint(
    "com_talos", "talos", robot.leftAnkle, com_la.tolist(), (True, True, True)
)

ps.createPositionConstraint(
    "gaze",
    "talos/rgbd_optical_joint",
    "box/root_joint",
    (0, 0, 0),
    (0, 0, 0),
    (True, True, False),
)

left_gripper_lock = []
right_gripper_lock = []
other_lock = ["talos/torso_1_joint"]
for n in robot.jointNames:
    s = robot.getJointConfigSize(n)
    r = robot.rankInConfiguration[n]
    if n.startswith("talos/gripper_right"):
        ps.createLockedJoint(n, n, half_sitting[r : r + s])
        right_gripper_lock.append(n)
    elif n.startswith("talos/gripper_left"):
        ps.createLockedJoint(n, n, half_sitting[r : r + s])
        left_gripper_lock.append(n)
    elif n in other_lock:
        ps.createLockedJoint(n, n, half_sitting[r : r + s])

graph = makeGraph(robot)

graph.setConstraints(
    graph=True,
    # lockDof = left_gripper_lock + right_gripper_lock + other_lock,
    numConstraints=foot_placement,
)
graph.initialize()

res, q_init, err = graph.generateTargetConfig("Loop | f", half_sitting, half_sitting)

ps.setParameter("SimpleTimeParameterization/safety", 0.5)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 2.0)

ps.setParameter("ConfigurationShooter/Gaussian/standardDeviation", 0.05)
ps.client.basic.problem.selectConfigurationShooter("Gaussian")

# Set initial guess for the current configuration
robot.setCurrentConfig(q_init)
