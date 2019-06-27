# /
import numpy as np
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraph, ProblemSolver, newProblem
from hpp.corbaserver.manipulation.robot import Robot
from hpp.gepetto.manipulation import ViewerFactory


newProblem()

Robot.packageName = "talos_data"
Robot.urdfName = "talos"
Robot.urdfSuffix = "_full"
Robot.srdfSuffix = ""

robot = Robot("dev", "talos", rootJointType="freeflyer")
robot.leftAnkle = "talos/leg_left_6_joint"
robot.rightAnkle = "talos/leg_right_6_joint"

robot.setJointBounds("talos/root_joint", [-1, 1, -1, 1, 0.5, 1.5])

ps = ProblemSolver(robot)
ps.selectPathProjector("Progressive", 0.2)
ps.setErrorThreshold(1e-3)
ps.setMaxIterProjection(40)

ps.addPathOptimizer("SimpleTimeParameterization")

vf = ViewerFactory(ps)

half_sitting = [
    0,
    0,
    1.0192720229567027,
    0,
    0,
    0,
    1,  # root_joint
    0.0,
    0.0,
    -0.411354,
    0.859395,
    -0.448041,
    -0.001708,  # leg_left
    0.0,
    0.0,
    -0.411354,
    0.859395,
    -0.448041,
    -0.001708,  # leg_right
    0,
    0.006761,  # torso
    0.25847,
    0.173046,
    -0.0002,
    -0.525366,
    0,
    0,
    0.1,  # arm_left
    0,
    0,
    0,
    0,
    0,
    0,
    0,  # gripper_left
    -0.25847,
    -0.173046,
    0.0002,
    -0.525366,
    0,
    0,
    0.1,  # arm_right
    0,
    0,
    0,
    0,
    0,
    0,
    0,  # gripper_right
    0,
    0,  # head
]
q_init = robot.getCurrentConfig()


ps.addPartialCom("talos", ["talos/root_joint"])

ps.createStaticStabilityConstraints(
    "balance", half_sitting, "talos", ProblemSolver.FIXED_ON_THE_GROUND
)
foot_placement = ["balance/pose-left-foot", "balance/pose-right-foot"]
foot_placement_complement = []

robot.setCurrentConfig(half_sitting)
com_wf = np.array(robot.getPartialCom("talos"))
tf_la = Transform(robot.getJointPosition(robot.leftAnkle))
com_la = tf_la.inverse().transform(com_wf)

ps.createRelativeComConstraint(
    "com_talos", "talos", robot.leftAnkle, com_la.tolist(), (True, True, True)
)

left_gripper_lock = []
right_gripper_lock = []
head_lock = []
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
    elif n.startswith("talos/head"):
        ps.createLockedJoint(n, n, half_sitting[r : r + s])
        head_lock.append(n)
    elif n in other_lock:
        ps.createLockedJoint(n, n, half_sitting[r : r + s])


graph = ConstraintGraph.buildGenericGraph(
    robot,
    "graph",
    ["talos/left_gripper", "talos/right_gripper"],
    [],
    [],
    [],  # contacts per object
    [],  # env contacts
    [],
)

graph.setConstraints(
    graph=True,
    lockDof=left_gripper_lock + right_gripper_lock + other_lock,
    numConstraints=[] + foot_placement,
)

graph.initialize()

q_init = half_sitting[:]


ps.setParameter("SimpleTimeParameterization/safety", 0.5)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)


# Create a relative pose constraint between the root_joint and the gripper we want to move
ps.createTransformationConstraint(
    "hand_pose_1",
    "talos/root_joint",
    "talos/gripper_right_joint",
    [0.3, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [True, True, True, False, False, False],
)

ps.resetConstraints()
ps.setNumericalConstraints("proj", foot_placement + ["hand_pose_1"])
ps.setLockedJointConstraints(
    "proj", left_gripper_lock + right_gripper_lock + other_lock
)

res, qproj1, err = ps.applyConstraints(q_init)


ps.createTransformationConstraint(
    "hand_pose_2",
    "talos/root_joint",
    "talos/gripper_right_joint",
    [0.6, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [True, True, True, False, False, False],
)

ps.resetConstraints()
ps.setNumericalConstraints("proj", foot_placement + ["hand_pose_2"])
ps.setLockedJointConstraints(
    "proj", left_gripper_lock + right_gripper_lock + other_lock
)

res, qproj2, err = ps.applyConstraints(q_init)


paths = list()
res, pid, msg = ps.directPath(q_init, qproj1, True)
paths.append(pid)
res, pid, msg = ps.directPath(qproj1, qproj2, True)
paths.append(pid)
res, pid, msg = ps.directPath(qproj2, q_init, True)
paths.append(pid)
# Apply to half_sitting
# Look if it's ok
#
# Do the same with the second position of the gripper
# directPath between the two ?
# Test with servo off
# Test
# Profit ?
