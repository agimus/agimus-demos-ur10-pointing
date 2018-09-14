#/usr/bin/env python
from hpp.corbaserver.manipulation.robot import Robot
from hpp.corbaserver.manipulation import newProblem, ProblemSolver, ConstraintGraph, Rule
from hpp.gepetto.manipulation import ViewerFactory
from hpp import Transform, Quaternion
import CORBA, sys, numpy as np
import random
from math import pi


newProblem()

Robot.packageName = "talos_data"
Robot.urdfName = "talos"
Robot.urdfSuffix = '_full'
Robot.srdfSuffix= ''

class Mire (object):
  rootJointType = 'freeflyer'
  packageName = 'agimus_demos'
  urdfName = 'calibration_mire'
  urdfSuffix = ""
  srdfSuffix = ""
  name = "mire"
  handles = [ "mire/left", "mire/right" ]

robot = Robot ('dev', 'talos', rootJointType = "freeflyer")
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
vf.loadObjectModel (Mire, 'mire')
robot.setJointBounds ("mire/root_joint", [-1, 1, -1, 1, 0, 2])

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

        0,0,0,0,0,0,1, # mire
        ]
q_init = robot.getCurrentConfig()

# INIT CHESSBOARD AND CAMERA PARAM AND FUNCTIONS
chessboard_pts = [[-0.05, -0.1, 0.0], [0.25, -0.1, 0.0], [0.25, 0.1, 0.0], [-0.05, 0.1, 0.0]]
chessboard_normal = np.matrix([0.0, 0.0, -1.0]).transpose()

image_width = 1280
image_height = 720
projection_matrix = np.matrix([[999.195, 0.0, 646.3244], [0.0, 1008.400, 359.955], [0.0, 0.0, 1.0]])

dist_from_camera = [0.3, 0.45]
camera_position = np.matrix([0., 0., 0.])

def isInImage( coord):
  x = coord[0,0]
  y = coord[1,0]
  return ( x >= 0 ) & ( x < image_width ) & ( y >= 0 ) & ( y < image_height )

def projectPoint( Rt, pt ):
  coord = projection_matrix * Rt * np.vstack(( pt, np.matrix([1]) ))
  coord /= coord[2,0]
  return coord[0:2,0]
# END INIT CHESSBOARD AND CAMERA PARAM AND FUNCTIONS


ps.addPartialCom ("talos", ["talos/root_joint"])
ps.addPartialCom ("talos_mire", ["talos/root_joint", "mire/root_joint"])

ps.createStaticStabilityConstraints ("balance", half_sitting, "talos", ProblemSolver.FIXED_ON_THE_GROUND)
foot_placement = [ "balance/pose-left-foot", "balance/pose-right-foot" ]
foot_placement_complement = [ ]

robot.setCurrentConfig(half_sitting)
com_wf = np.array(ps.getPartialCom("talos"))
tf_la = Transform (robot.getJointPosition(robot.leftAnkle))
com_la = tf_la.inverse().transform(com_wf)

ps.createRelativeComConstraint ("com_talos_mire", "talos_mire", robot.leftAnkle, com_la.tolist(), (True, True, True))
ps.createRelativeComConstraint ("com_talos"     , "talos"     , robot.leftAnkle, com_la.tolist(), (True, True, True))

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
        [ "mire", ],
        [ Mire.handles, ],
        [ [ ], ],
        [ ],
        [   Rule([ "talos/left_gripper", ], [ Mire.handles[0], ], True),
            Rule([ "talos/right_gripper", ], [ Mire.handles[1], ], True), ]
        )

graph.setConstraints (graph=True,
        lockDof = left_gripper_lock + right_gripper_lock + other_lock,
        numConstraints = [ "com_talos_mire"] + foot_placement)

graph.initialize()

robot.setCurrentConfig(half_sitting)
# Use the current robot velocity as std dev for the shooter 
# Higher values on the arm with the chessboard might limit the use of the other DoFs
ps.setParameter('ConfigurationShooter/Gaussian/useRobotVelocity', True)
robot.client.basic.robot.setCurrentVelocity([0.01,] * robot.getNumberDof())


q_proj_list = []
# Randomize the position of the chessboard
# The chessboard is on the right of the plate, so we shift the gaze (pointing to the centre of the plate) to the left
for x in [ 400, 450, 500, 550, 600, 650, 700 ]:
  for y in [ 300, 350, 400 ]:
    # Keep only poses where the chessboard can be seen from the camera
    while True:
      chessboard_Z = random.uniform( dist_from_camera[0], dist_from_camera[1] )
      chessboard_X = ( x - projection_matrix[0, 2] ) / projection_matrix[0, 0] * chessboard_Z 
      chessboard_Y = ( y - projection_matrix[1, 2] ) / projection_matrix[1, 1] * chessboard_Z 
      chessboard_position = np.matrix([ chessboard_X, chessboard_Y, chessboard_Z ])

      q = Quaternion().fromRPY( random.uniform( -pi/12., pi/12. ), random.uniform( -pi/12., pi/12. ), random.uniform( -pi/12., pi/12. ) ) 
      R = q.toRotationMatrix()
      if (R * chessboard_normal)[2] >= 0.:
        continue

      Rt = np.hstack(( R, ( chessboard_position - camera_position ).transpose() ))
      
      if not all( [ isInImage( projectPoint( Rt, np.matrix(pt).transpose() ) ) for pt in chessboard_pts ] ):
        continue
        
      q = Quaternion().fromRPY(-pi, 0, -pi) * q  # Switch tn the real camera frame
      
      chessboard_pose = (chessboard_position[0,0], chessboard_position[0,1], chessboard_position[0,2]) + q.toTuple()
      ps.createTransformationConstraint ("gaze", "talos/rgbd_rgb_optical_joint", "mire/root_joint", chessboard_pose, [True,]*6)
      ps.client.basic.problem.selectConfigurationShooter('Gaussian')

      ps.resetConstraints ()
      ps.setNumericalConstraints('proj', foot_placement + ["com_talos_mire", "talos/left_gripper grasps mire/left", "gaze", ])
      ps.setLockedJointConstraints('proj', left_gripper_lock + right_gripper_lock + other_lock)
      
      res, qproj, err = ps.applyConstraints(robot.shootRandomConfig())
      if res:
        print "Found pose"
        q_proj_list.append(qproj)
        break



res, q_init, err = graph.applyNodeConstraints("talos/left_gripper grasps mire/left", half_sitting)
res, q_goal, err = graph.applyNodeConstraints("talos/right_gripper grasps mire/right", half_sitting)


#ires, q, err = graph.generateTargetConfig(edge_name, config, config
# graph.edges pour liste des edges

print ps.directPath(q_init, q_init, True)
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)
ps.setParameter("SimpleTimeParameterization/safety", 0.5)
ps.setParameter("SimpleTimeParameterization/order", 2)

# ps.solve()
