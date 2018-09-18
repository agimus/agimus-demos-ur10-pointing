from common_hpp import *
from hpp.corbaserver.manipulation.robot import CorbaClient
from hpp.corbaserver.manipulation import ProblemSolver, Constraints
from hpp import Transform

import sys, numpy as np

footPlacement = True
comConstraint = True
constantWaistYaw = True
lockedTorso = False
fixedArmWhenGrasping = True

clients = CorbaClient (postContextId = "")
clients.manipulation.problem.resetProblem()

robot, ps, vf, table, objects = makeRobotProblemAndViewerFactory(clients)

q_neutral = robot.getCurrentConfig()

ps.addPartialCom ("talos", ["talos/root_joint"])
ps.addPartialCom ("talos_box", ["talos/root_joint", "box/root_joint"])

# Sliding static stability constraint
ps.createStaticStabilityConstraints ("balance", half_sitting, "talos",
                                     ProblemSolver.SLIDING)
foot_placement = [ "balance/relative-orientation",
                   "balance/relative-position",
                   "balance/orientation-left-foot",
                   "balance/position-left-foot" ]
foot_placement_complement = [ "balance/orientation-left-foot-complement",
                              "balance/position-left-foot-complement" ]

robot.setCurrentConfig(half_sitting)
# Position of COM with respect to left ankle
com_wf = np.array(ps.getPartialCom("talos"))
tf_la = Transform (robot.getJointPosition(robot.leftAnkle))
com_la = tf_la.inverse().transform(com_wf)

# COM constraints: robot and robot + box
ps.createRelativeComConstraint ("com_talos_box", "talos_box",
                                robot.leftAnkle, com_la.tolist(),
                                (True, True, True))
ps.createRelativeComConstraint ("com_talos", "talos", robot.leftAnkle,
                                com_la.tolist(), (True, True, True))

# Gaze constraint
ps.createPositionConstraint ("gaze", "talos/rgbd_optical_joint",
                             "box/root_joint", (0,0,0), (0,0,0),
                             (True, True, False))

# Constraint of constant yaw of the waist
ps.createOrientationConstraint ("waist_yaw", "", "talos/root_joint",
                                (0,0,0,1), [True, True, True])
ps.setConstantRightHandSide ("waist_yaw", False)

# Create lock joints for grippers
other_lock = list ()
# Create locked joint for torso
if lockedTorso:
    other_lock.append ("talos/torso_1_joint")

# lock position of table
other_lock.append (table.name + '/root_joint')

# Create locked joint for left arm
left_arm_lock = list ()
for n in robot.jointNames:
    if n.startswith ("talos/arm_left"):
        ps.createLockedJoint (n, n, [0,])
        ps.setConstantRightHandSide (n, False)
        left_arm_lock.append (n)

# Create locked joint for right arm
right_arm_lock = list ()
for n in robot.jointNames:
    if n.startswith ("talos/arm_right"):
        ps.createLockedJoint (n, n, [0,])
        ps.setConstantRightHandSide (n, False)
        right_arm_lock.append (n)

# Create locked joint for grippers
left_gripper_lock = list ()
right_gripper_lock = list ()
for n in robot.jointNames:
    s = robot.getJointConfigSize(n)
    r = robot.rankInConfiguration[n]
    if n.startswith ("talos/gripper_right"):
        ps.createLockedJoint(n, n, half_sitting[r:r+s])
        right_gripper_lock.append(n)
    elif n.startswith ("talos/gripper_left"):
        ps.createLockedJoint(n, n, half_sitting[r:r+s])
        left_gripper_lock.append(n)
    elif n in other_lock:
        ps.createLockedJoint(n, n, half_sitting[r:r+s])
        ps.setConstantRightHandSide (n, False)

# Set robot to neutral configuration before building constraint graph
robot.setCurrentConfig (q_neutral)
graph = makeGraph (robot, table, objects)

# On the real robot, the initial configuration as measured by sensors is very
# likely not in any state of the graph. State "starting_state" and transition
# "starting_motion" are aimed at coping with this issue.
graph.createNode ("starting_state")
graph.createEdge ("starting_state", "free", "starting_motion",
                  isInNode="starting_state")
graph.addConstraints (node="starting_state", constraints = \
                      Constraints (numConstraints = ["place_box"]) )
graph.addConstraints (edge="starting_motion", constraints = \
                      Constraints (numConstraints = ["place_box/complement"]))

# Add gaze and and COM constraints to each node of the graph
if comConstraint:
    for nodename, nodeid in graph.nodes.iteritems():
        graph.addConstraints (node=nodename, constraints =
                              Constraints (numConstraints =
                                           [ "com_talos_box", "gaze"]))

# Add locked joints and foot placement constraints in the graph,
# add foot placement complement in each edge.
if footPlacement:
    for edgename, edgeid in graph.edges.iteritems():
        graph.addConstraints (edge=edgename, constraints = Constraints \
                              (numConstraints = foot_placement_complement))

if constantWaistYaw:
    for edgename, edgeid in graph.edges.iteritems():
        graph.addConstraints (edge = edgename, constraints = Constraints
                              (numConstraints = ["waist_yaw"]))

graph.setConstraints (graph=True,
                      constraints = Constraints \
                      (numConstraints = foot_placement,
                       lockedJoints = left_gripper_lock + right_gripper_lock +\
                       other_lock))

# Transitions from state 'free'
e1 = 'talos/left_gripper > box/handle1 | f'
e2 = 'talos/right_gripper > box/handle3 | f'
e3 = 'talos/left_gripper > box/handle4 | f'
e4 = 'talos/right_gripper > box/handle2 | f'
# Transitions from one grasp to two grasps
e14 = 'talos/right_gripper > box/handle2 | 0-0'
e23 = 'talos/left_gripper > box/handle4 | 1-2'


# Transition from 'free' to first waypoint
e1_app = e1 + '_01'
e2_app = e2 + '_01'
e3_app = e3 + '_01'
e4_app = e4 + '_01'

if fixedArmWhenGrasping:
    leftArmConstraint = Constraints (lockedJoints = left_arm_lock)
    rightArmConstraint = Constraints (lockedJoints = right_arm_lock)

    graph.addConstraints (edge = e1_app, constraints = rightArmConstraint)
    graph.addConstraints (edge = e2_app, constraints = leftArmConstraint)
    graph.addConstraints (edge = e3_app, constraints = rightArmConstraint)
    graph.addConstraints (edge = e4_app, constraints = leftArmConstraint)
    graph.initialize ()


ps.setRandomSeed(123)
ps.selectPathProjector("Progressive", 0.2)
ps.selectPathValidation("Progressive", 0.001)
graph.initialize()

q_init = [0.5402763680625408, -0.833196863501999, 1.0199316910041052, -0.03128842007165536, 0.013789190720970665, 0.7297271046306221, 0.6828830395873728, -0.002517657851415276, 0.03462520266527989, -0.5316498053579248, 0.8402250533557625, -0.3730641123290547, -0.011780954381969872, -0.0025270209724267243, 0.034480300571697056, -0.5168007496652326, 0.8113706150231745, -0.3590584062316795, -0.011635750462120158, 0.0, 0.4392076095335054, 0.2806705510519144, 0.5, 0.0019674899062759165, -0.5194264855927397, 1.2349417194832937e-05, 0.0007850050683513623, 0.10090925286890041, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2814831804277627, -0.5, -0.004238959829568303, -0.5200522586579716, 0.00014996678886283413, -0.0015425422291322729, 0.10092910629223316, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5143008291852817, 0.0455661913503581, 0.45891797741593393, -0.25, 0.832, -0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0, 0, 1]

# Set Gaussian configuration shooter.
robot.setCurrentConfig (q_init)
sigma = robot.getNumberDof () * [.1]
rank = robot.rankInVelocity [robot.displayName + '/root_joint']
sigma [rank:rank+6] = 6* [0.]
rank = robot.rankInVelocity [objects [0].name + '/root_joint']
sigma [rank:rank+6] = 6* [0.05]
robot.setCurrentVelocity (sigma)
ps.setParameter ('ConfigurationShooter/Gaussian/useRobotVelocity', True)
ps.client.basic.problem.selectConfigurationShooter ('Gaussian')
# Set Optimization parameters
ps.setParameter("SimpleTimeParameterization/safety", 0.5)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 2.)

q_init = robot.shootRandomConfig ()
# Define problem
res, q_init, err = graph.applyNodeConstraints ('free', q_init)
if not res: raise RuntimeError ('Failed to project initial configuration')

q_goal = q_init [::]
rank = robot.rankInConfiguration ['box/root_joint']
q_goal [rank+3:rank+7] = [-0.5, -0.5, -0.5, 0.5]

solver = Solver (ps, graph, q_init, q_goal, e1, e2, e3, e4, e14, e23)

