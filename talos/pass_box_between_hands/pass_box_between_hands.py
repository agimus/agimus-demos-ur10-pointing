#/usr/bin/env python
from hpp.corbaserver.manipulation.robot import Robot
from hpp.corbaserver.manipulation import newProblem, ProblemSolver, ConstraintGraph, Rule
from hpp.gepetto.manipulation import ViewerFactory
from hpp import Transform
import CORBA, sys, numpy as np

newProblem()

Robot.packageName = "talos-data"
Robot.urdfName = "talos"
Robot.urdfSuffix = '_full'
Robot.srdfSuffix= ''

class Box (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_tutorial'
  meshPackageName = 'hpp_tutorial'
  urdfName = 'cup'
  urdfSuffix = ""
  srdfSuffix = ""
  handles = [ "box/top", "box/bottom" ]

class Brick (object):
  rootJointType = 'freeflyer'
  packageName = 'gerard-bauzil'
  urdfName = 'cobblestone'
  urdfSuffix = ""
  srdfSuffix = ""
  handles = [ "box/handle1", "box/handle2" ]

class Env (object):
  packageName = 'gerard-bauzil'
  urdfName = 'staircases_koroibot'
  urdfSuffix = ""
  srdfSuffix = ""

class Table (object):
  packageName = 'gerard-bauzil'
  urdfName = 'pedestal_table'
  urdfSuffix = ""
  srdfSuffix = ""
  pose = "pose"

robot = Robot ('dev', 'talos', rootJointType = "freeflyer")
robot. leftAnkle = "talos/leg_left_6_joint"
robot.rightAnkle = "talos/leg_right_6_joint"

robot.setJointBounds ("talos/root_joint", [-1, 1, -1, 1, 0.5, 1.5])

ps = ProblemSolver (robot)
ps.selectPathProjector("Progressive", 0.2)
ps.setErrorThreshold (1e-3)
ps.setMaxIterProjection (40)

ps.addPathOptimizer("SimpleTimeParameterization")
# ps.setParameter("SplineGradientBased/convertPathToSpline", CORBA.Any(CORBA.TC_boolean, True))
# ps.addPathOptimizer("SplineGradientBased_bezier3")

vf = ViewerFactory (ps)
Object = Box
vf.loadObjectModel (Object, 'box')
# vf.loadObjectModel (Brick, 'box')
robot.setJointBounds ("box/root_joint", [-1, 1, -1, 1, 0, 2])

qq = [-0.7671778026566639, 0.0073267002287253635, 1.0035168727631776, -0.003341673452654457, -0.021566597515109524, 0.0002183620894239602, 0.9997618053357284, -0.00020053128587844821, -0.0021365695604276275, -0.4415951773681094, 0.9659230706255528, -0.48119003672520416, 0.007109157982067145, -0.00020095991543181877, -0.002126639473414498, -0.4382848597339842, 0.9589221865248464, -0.4774994711722908, 0.007099218648561522, 0.0, 0.025235347910697536, -0.06985947194357875, 0.0, -0.12446173084176845, -1.5808415926365578, 0.014333078135875619, -0.0806417043955706, -0.37124401660668394, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25955282922987977, -0.1618313202464181, 0.002447883426630002, -0.5149037074691503, -0.00010703274362664899, 0.0008742582163227642, 0.10168585913285667, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.785398163397, 0.32250041955468695, -0.2569883469655496, 0.18577095561452217, 1.164388709412583, 0.0694401264431558, 0.5475575114527793, -0.11842286843715424, 0.8254301089264399]

half_sitting = [
        0,0,1.0192720229567027,0,0,0,1, # root_joint
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_left
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_right
        0, 0.006761, # torso
        0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, # arm_left
        0, 0, 0, 0, 0, 0, 0, # gripper_left
        -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, # arm_right
        0, 0, 0, 0, 0, 0, 0, # gripper_right
        0, 0, # head

        0,0,0,0,0,0,1, # box
        ]
qqq = qq[:7] + half_sitting[7:-7] + qq[-7:]
# ps.selectSteeringMethod('Straight')
# ps.setParameter("SimpleTimeParameterization/safety", CORBA.Any(CORBA.TC_double, 0.5))
# ps.setParameter("SimpleTimeParameterization/velocity", CORBA.Any(CORBA.TC_boolean, True))
# print ps.directPath(qqq, qq, False)
# #ps.addPathOptimizer("SimpleTimeParameterization")
# ps.optimizePath(0)
# #ps.clearPathOptimizers()
# ps.selectSteeringMethod('Graph-Straight')
q_init = robot.getCurrentConfig()

# vf.loadEnvironmentModel (Env, 'env')
# vf.loadEnvironmentModel (Table, 'table')
half_sitting[0] = -0.74

ps.addPartialCom ("talos", ["talos/root_joint"])
ps.addPartialCom ("talos_box", ["talos/root_joint", "box/root_joint"])

# ps.createStaticStabilityConstraints ("balance", half_sitting, "talos")
# foot_placement = [ "balance/relative-orientation", "balance/relative-position", "balance/orientation-left-foot", "balance/position-left-foot", ]
# foot_placement_complement = [ "balance/orientation-left-foot-complement", "balance/position-left-foot-complement" ]
ps.createStaticStabilityConstraints ("balance", half_sitting, "talos", ProblemSolver.FIXED_ON_THE_GROUND)
foot_placement = [ "balance/pose-left-foot", "balance/pose-right-foot" ]
foot_placement_complement = [ ]

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
        right_gripper_lock.append(n)
    elif n.startswith ("talos/gripper_left"):
        ps.createLockedJoint(n, n, half_sitting[r:r+s])
        left_gripper_lock.append(n)
    elif n in other_lock:
        ps.createLockedJoint(n, n, half_sitting[r:r+s])

graph = ConstraintGraph.buildGenericGraph(robot, 'graph',
        [ "talos/left_gripper", "talos/right_gripper", ],
        [ "box", ],
        [ Object.handles, ],
        [ [ ], ],
        [ ],
        [ Rule([ "talos/left_gripper", ], [ Object.handles[0], ], True),
            Rule([ "talos/right_gripper", ], [ Object.handles[1], ], True), ]
        )

graph.setConstraints (graph=True,
        lockDof = left_gripper_lock + right_gripper_lock + other_lock,
        numConstraints = [ "com_talos_box", "gaze"] + foot_placement)
# graph.createNode (['free'], False, [1, ])
graph.initialize()

res, q_init, err = graph.applyNodeConstraints("talos/left_gripper grasps box/top", half_sitting)
res, q_goal, err = graph.applyNodeConstraints("talos/right_gripper grasps box/bottom", half_sitting)
print ps.directPath(q_init, q_init, True)
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)
ps.setParameter("SimpleTimeParameterization/safety", CORBA.Any(CORBA.TC_double, 0.5))
ps.setParameter("SimpleTimeParameterization/velocity", CORBA.Any(CORBA.TC_boolean, True))

ps.solve()

import sys
sys.exit(0)

# graph = ConstraintGraph.buildGenericGraph(robot, 'graph',
        # [ "table/pose", "talos/left_gripper", "talos/right_gripper", ],
        # [ "box", ],
        # [ [ "box/handle1", "box/handle2", "box/pose1", "box/pose2"], ],
        # [ [ ], ],
        # [ ],
        # [ Rule([ "table/pose", ], [ "box/handle[12]", ], False),
            # Rule([ "talos/right_gripper", ], [ "box/handle2", ], False),
            # Rule([ "talos/left_gripper", ], [ "box/handle1", ], False),
            # # Rule([ "table/pose", ], [ "box/handle1", ], False),
            # Rule([ "talos/right_gripper", ], [ "box/pose[12]", ], False),
            # Rule([ "talos/left_gripper", ], [ "box/pose[12]", ], False),
            # Rule([ "table/pose", "talos/right_gripper"], [ "box/pose1", "box/handle1" ], False),
            # Rule([ "table/pose", "talos/left_gripper"], [ "box/pose2", "box/handle2" ], False),
            # Rule([ "table/pose", ], [ "box/pose.*", ], True),
            # Rule([ "table/pose", "talos/left_gripper" ], [ "", "box/handle[12]" ], True),
            # Rule([ "table/pose", "talos/right_gripper" ], [ "", "box/handle[12]" ], True),
            # Rule([ "talos/right_gripper", ], [ "box/handle2", ], True),
            # Rule([ "talos/left_gripper", ], [ "box/handle1", ], True), ])
# graph.setConstraints (graph=True,
        # lockDof = left_gripper_lock + right_gripper_lock,
        # numConstraints = [ "com_talos_box", "gaze"] + foot_placement)


grippers = ['talos/right_gripper', 'talos/left_gripper', 'table/pose']
objects = ['box']
handlesPerObject = [ ['box/pose1', 'box/pose2', 'box/handle1', 'box/handle2']  ]
contactsPerObject = [ [] ]
rules = [
    Rule(['table/pose', 'talos/right_gripper', 'talos/left_gripper'], ['box/pose[12]', '', ''], True),
    Rule(['table/pose', 'talos/right_gripper', 'talos/left_gripper'], ['box/pose1', '', 'box/handle2'], True),
    Rule(['table/pose', 'talos/right_gripper', 'talos/left_gripper'], ['box/pose2', 'box/handle1', ''], True),
    Rule(['table/pose', 'talos/right_gripper', 'talos/left_gripper'], ['', 'box/handle1', ''], True),
    Rule(['table/pose', 'talos/right_gripper', 'talos/left_gripper'], ['', 'box/handle1', 'box/handle2'], True),
    Rule(['table/pose', 'talos/right_gripper', 'talos/left_gripper'], ['', '', 'box/handle2'], True),
    ]

graph = ConstraintGraph.buildGenericGraph(robot, "my_graph", grippers, objects, handlesPerObject, contactsPerObject, [], rules)

graph.createNode("free")
graph.setContainingNode( "talos/left_gripper > box/handle2 | 2-0", "free")
graph.setContainingNode("talos/right_gripper > box/handle1 | 2-1", "free")

# Try locking the head and making some edge short.
graph.setConstraints (graph=True,
        # lockDof = left_gripper_lock + right_gripper_lock + other_lock,
        lockDof = left_gripper_lock + right_gripper_lock,
        # numConstraints = [ "gaze"] + foot_placement)
        numConstraints = foot_placement)

for n, id in graph.edges.iteritems():
    graph.setConstraints (edge = n, numConstraints = foot_placement_complement)

node_list_talos_box = [
    'talos/left_gripper grasps box/handle2 : table/pose grasps box/pose1',
    'talos/right_gripper grasps box/handle1 : table/pose grasps box/pose2',
    'talos/left_gripper grasps box/handle2',
    'talos/right_gripper grasps box/handle1 : talos/left_gripper grasps box/handle2',
    'talos/right_gripper grasps box/handle1']

node_list_talos = ['table/pose grasps box/pose1',
        'table/pose grasps box/pose2',
        'talos/left_gripper > box/handle2 | 2-0_pregrasp',
        "talos/right_gripper > box/handle1 | 2-1_pregrasp"
        ]

for n in node_list_talos_box :
    graph.setConstraints (node = n, numConstraints = [ "com_talos_box" ] )

for n in node_list_talos:
    graph.setConstraints (node = n, numConstraints = [ "com_talos" ] )

edge_to_disable = [
        "talos/right_gripper < box/handle1 | 0-2:2-1",
        "table/pose > box/pose2 | 0-2",
        "table/pose > box/pose1 | 1-3",
        "talos/left_gripper < box/handle2 | 1-3:2-0",
        ]

for e in edge_to_disable: graph.setWeight(e, 0)

res1, q1, err1 = graph.applyNodeConstraints("table/pose grasps box/pose2", half_sitting)
res2, q2, err2 = graph.applyNodeConstraints("table/pose grasps box/pose1", half_sitting)
ps.setInitialConfig(q1)
ps.addGoalConfig(q2)

ps.solve()
