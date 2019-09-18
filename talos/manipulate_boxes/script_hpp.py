import numpy as np
from hpp import Quaternion, Transform
from hpp.corbaserver.manipulation import Constraints, ProblemSolver
from hpp.corbaserver.manipulation.robot import CorbaClient
from hpp.corbaserver import loadServerPlugin

from common_hpp import *

loadServerPlugin ("corbaserver", "manipulation-corba.so")


footPlacement = True
comConstraint = True
constantWaistYaw = True
fixedArmWhenGrasping = True

client = CorbaClient(context="corbaserver")
client.manipulation.problem.resetProblem()

robot, ps, vf, table, objects = makeRobotProblemAndViewerFactory(client)

q_neutral = robot.getCurrentConfig()

ps.addPartialCom("talos", ["talos/root_joint"])
ps.addPartialCom("talos_box", ["talos/root_joint", "box/root_joint"])

# Static stability constraint
robot.createStaticStabilityConstraint(
    "balance/", "talos", robot.leftAnkle, robot.rightAnkle, init_conf
)
foot_placement = ["balance/pose-left-foot", "balance/pose-right-foot"]
foot_placement_complement = []

# Static stability constraint with box
robot.createStaticStabilityConstraint(
    "balance_box/", "talos_box", robot.leftAnkle, robot.rightAnkle, init_conf
)


# Gaze constraint
ps.createPositionConstraint(
    "gaze",
    "talos/rgbd_optical_joint",
    "box/root_joint",
    (0, 0, 0),
    (0, 0, 0),
    (True, True, False),
)

# Constraint of constant yaw of the waist
ps.createOrientationConstraint(
    "waist_yaw", "", "talos/root_joint", (0, 0, 0, 1), [True, True, True]
)
ps.setConstantRightHandSide("waist_yaw", False)

# Create lock joints for grippers
table_lock = list()

# lock position of table
table_lock.append(table.name + "/root_joint")

# Create locked joint for left arm
left_arm_lock = list()
for n in robot.jointNames:
    if n.startswith("talos/arm_left"):
        ps.createLockedJoint(n, n, [0])
        ps.setConstantRightHandSide(n, False)
        left_arm_lock.append(n)

# Create locked joint for right arm
right_arm_lock = list()
for n in robot.jointNames:
    if n.startswith("talos/arm_right"):
        ps.createLockedJoint(n, n, [0])
        ps.setConstantRightHandSide(n, False)
        right_arm_lock.append(n)

# Create locked joint for grippers
left_gripper_lock = list()
right_gripper_lock = list()
for n in robot.jointNames:
    s = robot.getJointConfigSize(n)
    r = robot.rankInConfiguration[n]
    if n.startswith("talos/gripper_right"):
        ps.createLockedJoint(n, n, init_conf[r : r + s])
        right_gripper_lock.append(n)
    elif n.startswith("talos/gripper_left"):
        ps.createLockedJoint(n, n, init_conf[r : r + s])
        left_gripper_lock.append(n)
    elif n in table_lock:
        ps.createLockedJoint(n, n, init_conf[r : r + s])
        ps.setConstantRightHandSide(n, False)

# Set robot to neutral configuration before building constraint graph
robot.setCurrentConfig(q_neutral)
graph = makeGraph(robot, table, objects)

# Add other locked joints in the edges.
for edgename, edgeid in graph.edges.iteritems():
    graph.addConstraints(
        edge=edgename, constraints=Constraints(numConstraints=table_lock)
    )
# Add gaze and and COM constraints to each node of the graph
if comConstraint:
    for nodename, nodeid in graph.nodes.iteritems():
        graph.addConstraints(
            node=nodename, constraints=Constraints(numConstraints=["balance/relative-com", "gaze"])
        )

# Add locked joints and foot placement constraints in the graph,
# add foot placement complement in each edge.
if footPlacement:
    for edgename, edgeid in graph.edges.iteritems():
        graph.addConstraints(
            edge=edgename,
            constraints=Constraints(numConstraints=foot_placement_complement),
        )

if constantWaistYaw:
    for edgename, edgeid in graph.edges.iteritems():
        graph.addConstraints(
            edge=edgename, constraints=Constraints(numConstraints=["waist_yaw"])
        )

graph.addConstraints(
    graph=True,
    constraints=Constraints(
        numConstraints=foot_placement + left_gripper_lock + right_gripper_lock,
    ),
)

# On the real robot, the initial configuration as measured by sensors is very
# likely not in any state of the graph. State "starting_state" and transition
# "starting_motion" are aimed at coping with this issue.
graph.createNode("starting_state")
graph.createEdge("starting_state", "free", "starting_motion", isInNode="starting_state")
graph.createEdge(
    "starting_state", "starting_state", "loop_ss", isInNode="starting_state", weight=0
)
graph.createEdge(
    "free",
    "starting_state",
    "go_to_starting_state",
    isInNode="starting_state",
    weight=0,
)
graph.addConstraints(
    node="starting_state", constraints=Constraints(numConstraints=["place_box"])
)
graph.addConstraints(
    edge="loop_ss",
    constraints=Constraints(
        numConstraints=["place_box/complement"] + table_lock
    ),
)
graph.addConstraints(
    edge="starting_motion",
    constraints=Constraints(
        numConstraints=["place_box/complement"] + table_lock
    ),
)
graph.addConstraints(
    edge="go_to_starting_state",
    constraints=Constraints(
        numConstraints=["place_box/complement"] + table_lock
    ),
)

# Transitions from state 'free'
e_l1 = "talos/left_gripper > box/handle1 | f"
e_r1 = "talos/right_gripper > box/handle1 | f"
e_l2 = "talos/left_gripper > box/handle2 | f"
e_r2 = "talos/right_gripper > box/handle2 | f"
e_l3 = "talos/left_gripper > box/handle3 | f"
e_r3 = "talos/right_gripper > box/handle3 | f"
e_l4 = "talos/left_gripper > box/handle4 | f"
e_r4 = "talos/right_gripper > box/handle4 | f"
# Transitions from one grasp to two grasps
e_l1_r2 = "talos/right_gripper > box/handle2 | 0-0"
e_l1_r4 = "talos/right_gripper > box/handle4 | 0-0"
e_r1_l2 = "talos/left_gripper > box/handle2 | 1-0"
e_r1_l4 = "talos/left_gripper > box/handle4 | 1-0"
e_l2_r1 = "talos/right_gripper > box/handle1 | 0-3"
e_l2_r3 = "talos/right_gripper > box/handle3 | 0-3"
e_r2_l1 = "talos/left_gripper > box/handle3 | 1-3"
e_r2_l3 = "talos/left_gripper > box/handle3 | 1-3"
e_r3_l4 = "talos/left_gripper > box/handle4 | 1-2"
e_r3_l2 = "talos/left_gripper > box/handle2 | 1-2"
e_l3_r4 = "talos/right_gripper > box/handle4 | 0-2"
e_l3_r2 = "talos/right_gripper > box/handle2 | 0-2"
e_l4_r1 = "talos/right_gripper > box/handle1 | 0-3"
e_l4_r3 = "talos/right_gripper > box/handle3 | 0-3"
e_r4_l1 = "talos/left_gripper > box/handle1 | 1-3"
e_r4_l3 = "talos/left_gripper > box/handle3 | 1-3"
# Transition from 'free' to first waypoint
e_l1_app = e_l1 + "_01"
e_r1_app = e_r1 + "_01"
e_l2_app = e_l2 + "_01"
e_r2_app = e_r2 + "_01"
e_l3_app = e_l3 + "_01"
e_r3_app = e_r3 + "_01"
e_l4_app = e_l4 + "_01"
e_r4_app = e_r4 + "_01"

if fixedArmWhenGrasping:
    leftArmConstraint = Constraints(numConstraints=left_arm_lock)
    rightArmConstraint = Constraints(numConstraints=right_arm_lock)

    graph.addConstraints(edge=e_l1_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r1_app, constraints=leftArmConstraint)
    graph.addConstraints(edge=e_l2_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r2_app, constraints=leftArmConstraint)
    graph.addConstraints(edge=e_l3_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r3_app, constraints=leftArmConstraint)
    graph.addConstraints(edge=e_l4_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r4_app, constraints=leftArmConstraint)
    graph.initialize()


ps.setRandomSeed(123)
ps.selectPathProjector("Progressive", 0.2)
# ps.selectPathValidation("Progressive", 0.01)
ps.selectPathValidation("Discretized", 0.01)
# ps.selectPathValidation("Dichotomy", 0.0)
graph.initialize()

q_init = [
    0.5402763680625408,
    -0.833196863501999,
    1.0199316910041052,
    -0.03128842007165536,
    0.013789190720970665,
    0.7297271046306221,
    0.6828830395873728,
    -0.002517657851415276,
    0.03462520266527989,
    -0.5316498053579248,
    0.8402250533557625,
    -0.3730641123290547,
    -0.011780954381969872,
    -0.0025270209724267243,
    0.034480300571697056,
    -0.5168007496652326,
    0.8113706150231745,
    -0.3590584062316795,
    -0.011635750462120158,
    0.0,
    0.4392076095335054,
    0.2806705510519144,
    0.5,
    0.0019674899062759165,
    -0.5194264855927397,
    1.2349417194832937e-05,
    0.0007850050683513623,
    0.10090925286890041,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -0.2814831804277627,
    -0.5,
    -0.004238959829568303,
    -0.5200522586579716,
    0.00014996678886283413,
    -0.0015425422291322729,
    0.10092910629223316,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.5143008291852817,
    0.0455661913503581,
    0.45891797741593393,
    -0.25,
    0.832,
    -0.5,
    0.5,
    0.5,
    0.5,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
]

# Set Gaussian configuration shooter.
robot.setCurrentConfig(q_init)
# Set variance to 0.1 for all degrees of freedom
sigma = robot.getNumberDof() * [0.1]
# Set variance to 0.05 for robot free floating base
rank = robot.rankInVelocity[robot.displayName + "/root_joint"]
sigma[rank : rank + 6] = 6 * [0.0]
# Set variance to 0.05 for box
rank = robot.rankInVelocity[objects[0].name + "/root_joint"]
sigma[rank : rank + 6] = 6 * [0.0]
# Set variance to 0.05 for table
rank = robot.rankInVelocity[table.name + "/root_joint"]
sigma[rank : rank + 6] = 6 * [0.0]
robot.setCurrentVelocity(sigma)
ps.setParameter("ConfigurationShooter/Gaussian/useRobotVelocity", True)
ps.client.basic.problem.selectConfigurationShooter("Gaussian")
q_init[robot.rankInConfiguration["box/root_joint"] + 1] += 0.1
# Set Optimization parameters
ps.setParameter("SimpleTimeParameterization/safety", 0.25)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)
ps.setParameter("ManipulationPlanner/extendStep", 0.7)
ps.setMaxIterPathPlanning(50)

# q_init = robot.shootRandomConfig ()
# Define problem
res, q_init, err = graph.generateTargetConfig("Loop | f", q_init, q_init)
if not res:
    raise RuntimeError("Failed to project initial configuration")

q_goal = q_init[::]
rank = robot.rankInConfiguration["box/root_joint"]
q_goal[rank + 3 : rank + 7] = (
    Quaternion([0, 1, 0, 0]) * Quaternion(q_init[rank + 3 : rank + 7])
).toTuple()
# res, q_goal, err = graph.applyNodeConstraints ('free', q_goal)
res, q_proj, err = graph.generateTargetConfig("Loop | f", q_goal, q_goal)
if not res:
    raise RuntimeError("Failed to project goal configuration")
assert q_init[-7:] == q_goal[-7:]

solver = Solver(
    ps,
    graph,
    q_init,
    q_goal,
    e_l1,
    e_r1,
    e_l2,
    e_r2,
    e_l3,
    e_r3,
    e_l4,
    e_r4,
    e_l1_r2,
    e_l1_r4,
    e_r1_l2,
    e_r1_l4,
    e_l2_r1,
    e_l2_r3,
    e_r2_l1,
    e_r2_l3,
    e_r3_l4,
    e_r3_l2,
    e_l3_r4,
    e_l3_r2,
    e_l4_r1,
    e_l4_r3,
    e_r4_l1,
    e_r4_l3,
)

solver.initRosNode()

qBoxVisible, pathId = solver.makeBoxVisibleFrom(init_conf, True, True)

# From an estimated configuration with position of objects
# solver.solveFromEstimatedConfiguration (init_conf)

## Solving with ManipulationRRT and random shortcut takes approximately 2 minutes
# ps.setMaxIterPathPlanning(1000)
# ps.clearPathOptimizers ()
# ps.addPathOptimizer ("RandomShortcut")
# ps.addPathOptimizer ("SimpleTimeParameterization")
# ps.setInitialConfig (q_init)
# ps.addGoalConfig (q_goal)
# time = ps.solve ()
