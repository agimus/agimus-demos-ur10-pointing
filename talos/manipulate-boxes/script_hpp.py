from common_hpp import *
from hpp.corbaserver.manipulation.robot import CorbaClient
from hpp.corbaserver.manipulation import ProblemSolver, Constraints
from hpp import Transform

import sys, numpy as np

footPlacement = True
comConstraint = True

clients = CorbaClient (postContextId = "")
clients.manipulation.problem.resetProblem()

robot, ps, vf, table, objects = makeRobotProblemAndViewerFactory(clients)

q_neutral = robot.getCurrentConfig()

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

# Create lock joints for grippers
left_gripper_lock = []
right_gripper_lock = []
other_lock = ["talos/torso_1_joint"]
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

# Set robot to neutral configuration before building constraint graph
robot.setCurrentConfig (q_neutral)
graph = makeGraph (robot, table, objects)

if comConstraint:
    for nodename, nodeid in graph.nodes.iteritems():
        graph.addConstraints (node=nodename, constraints =
                              Constraints (numConstraints =
                                           [ "com_talos_box", "gaze"]))

graph.createNode("starting_state")
graph.createEdge("starting_state", "free", "starting_motion", isInNode="starting_state")
graph.addConstraints(node="starting_state",
	numConstraints = ["place_box", ] )
graph.addConstraints(edge="starting_motion",
	numConstraints = ["place_box/complement", ] )

if footPlacement:
    for edgename, edgeid in graph.edges.iteritems():
        graph.addConstraints (edge=edgename,
                              numConstraints = foot_placement_complement)

    graph.setConstraints (graph=True,
                          lockDof = left_gripper_lock + right_gripper_lock\
                          + other_lock,
                          numConstraints = foot_placement)
ps.setRandomSeed(123)
ps.selectPathProjector("Progressive", 0.2)
ps.selectPathValidation("Progressive", 0.01)
graph.initialize()

q_init = [0.5402763680625408, -0.833196863501999, 1.0199316910041052, -0.03128842007165536, 0.013789190720970665, 0.7297271046306221, 0.6828830395873728, -0.002517657851415276, 0.03462520266527989, -0.5316498053579248, 0.8402250533557625, -0.3730641123290547, -0.011780954381969872, -0.0025270209724267243, 0.034480300571697056, -0.5168007496652326, 0.8113706150231745, -0.3590584062316795, -0.011635750462120158, 0.0, 0.4392076095335054, 0.2806705510519144, 0.18306255865650514, 0.0019674899062759165, -0.5194264855927397, 1.2349417194832937e-05, 0.0007850050683513623, 0.10090925286890041, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2814831804277627, -0.19465310763470348, -0.004238959829568303, -0.5200522586579716, 0.00014996678886283413, -0.0015425422291322729, 0.10092910629223316, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5143008291852817, 0.0455661913503581, 0.45891797741593393, -0.19025700845205984, 0.832, 0.0, a, 0.0, a]

# Set Gaussian configuration shooter.
robot.setCurrentConfig (q_init)
sigma = robot.getNumberDof () * [.05]
rank = robot.rankInVelocity [robot.displayName + '/root_joint']
sigma [rank:rank+6] = 6* [0.]
robot.setCurrentVelocity (sigma)
ps.setParameter ('ConfigurationShooter/Gaussian/useRobotVelocity', True)
ps.client.basic.problem.selectConfigurationShooter ('Gaussian')
# Set Optimization parameters
ps.setParameter("SimpleTimeParameterization/safety", 0.5)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 2.)

# Define problem
res, q_init, err = graph.applyNodeConstraints ('free', q_init)
if not res: raise RuntimeError ('Failed to project initial configuration')

q_goal = q_init [::]
q_goal [-4:] = [0.0, -a, 0.0, a]

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

#ps.solve ()
