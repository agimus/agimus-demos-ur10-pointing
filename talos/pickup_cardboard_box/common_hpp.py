#/usr/bin/env python
from hpp.corbaserver.manipulation.robot import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule
from hpp.gepetto.manipulation import ViewerFactory

Robot.packageName = "talos_data"
Robot.urdfName = "talos"
Robot.urdfSuffix = '_full_v2'
Robot.srdfSuffix= ''

class Box (object):
  rootJointType = 'freeflyer'
  packageName = 'gerard_bauzil'
  urdfName = 'cardboard_box'
  urdfSuffix = ""
  srdfSuffix = ""
  handles = [ "box/handle1", "box/handle2" ]
  contacts = [ "box/bottom_surface", ]

class Table (object):
  rootJointType = 'anchor'
  packageName = 'gerard_bauzil'
  urdfName = 'pedestal_table'
  urdfSuffix = ""
  srdfSuffix = ""
  pose = "pose"
  contacts = [ "table/support", ]

Object = Box
half_sitting = [
        # -0.74,0,1.0192720229567027,0,0,0,1, # root_joint
        -0.6,-0.2,1.0192720229567027,0,0,0,1, # root_joint
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_left
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_right
        0, 0.006761, # torso
        0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, # arm_left
        0, 0, 0, 0, 0, 0, 0, # gripper_left
        -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, # arm_right
        0, 0, 0, 0, 0, 0, 0, # gripper_right
        0, 0, # head

        -0.04,0,1.095+0.071,0,0,1,0, # box
        ]

def makeRobotProblemAndViewerFactory (clients):
    robot = Robot ('talos', 'talos', rootJointType = "freeflyer", client = clients)
    robot. leftAnkle = "talos/leg_left_6_joint"
    robot.rightAnkle = "talos/leg_right_6_joint"

    robot.setJointBounds ("talos/root_joint", [-1, 1, -1, 1, 0.5, 1.5])

    ps = ProblemSolver (robot)
    ps.setRandomSeed(123)
    ps.selectPathProjector("Progressive", 0.2)
    ps.setErrorThreshold (1e-3)
    ps.setMaxIterProjection (40)

    ps.addPathOptimizer("SimpleTimeParameterization")

    vf = ViewerFactory (ps)
    vf.loadObjectModel (Object, 'box')
    robot.setJointBounds ("box/root_joint", [-1, 1, -1, 1, 0, 2])

    # Loaded as an object to get the visual tags at the right position.
    # vf.loadEnvironmentModel (Table, 'table')
    vf.loadObjectModel (Table, 'table')

    return robot, ps, vf

def makeGraph (robot):
    from hpp.corbaserver.manipulation.constraint_graph_factory import ConstraintGraphFactory
    graph = ConstraintGraph(robot, 'graph')
    factory = ConstraintGraphFactory (graph)
    factory.setGrippers ([ "talos/left_gripper", ])
    factory.setObjects ([ "box", ],
            [ Object.handles, ],
            [ Object.contacts, ])
    factory.environmentContacts (Table.contacts)
    factory.setRules (
            [
              Rule([ "talos/left_gripper", ], [ Object.handles[1], ], False),
              # Rule([ "talos/left_gripper", ], [ Object.handles[0], ], True),
              Rule([ "talos/left_gripper", ], [ ".*", ], True),
              # Rule([ "talos/right_gripper", ], [ Object.handles[1], ], True),
              ]
            )
    factory.generate ()
    return graph
