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
Robot.urdfSuffix = '_full_v2'
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
head_lock = []
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
    elif n.startswith ("talos/head"):
        ps.createLockedJoint(n, n, half_sitting[r:r+s])
        head_lock.append(n)
    elif n in other_lock:
        ps.createLockedJoint(n, n, half_sitting[r:r+s])

graph = ConstraintGraph.buildGenericGraph(robot, 'graph',
        [ "talos/left_gripper", "talos/right_gripper", ],
        [ "mire", ],
        [ Mire.handles, ],
        [ [ ], ], # contacts per object
        [ ], # env contacts
        [   Rule([ "talos/left_gripper", ], [ Mire.handles[0], ], True),
            Rule([ "talos/right_gripper", ], [ Mire.handles[1], ], True), ]
        )

graph.setConstraints (graph=True,
        lockDof = left_gripper_lock + right_gripper_lock + other_lock,
        numConstraints = [ "com_talos_mire"] + foot_placement)

graph.initialize()

def setGuassianShooter (mean, stddev = [0.01,] * robot.getNumberDof()):
    robot.setCurrentConfig(mean)
    # Use the current robot velocity as std dev for the shooter 
    # Higher values on the arm with the chessboard might limit the use of the other DoFs
    ps.setParameter('ConfigurationShooter/Gaussian/useRobotVelocity', True)
    robot.client.basic.robot.setCurrentVelocity(stddev)
    ps.client.basic.problem.selectConfigurationShooter('Gaussian')

qrand = half_sitting[:]
q_proj_list = []

expected_poses = []

# Randomize the position of the chessboard
# The chessboard is on the right of the plate, so we shift the gaze (pointing to the centre of the plate) to the left
ys = [ 300, 350, 400, 450 ]
for x in [ 400, 450, 500, 550, 600, 650, 700, 750, 800 ]:
  for y in ys:
    # Keep only poses where the chessboard can be seen from the camera
    setGuassianShooter (qrand)
    shoot_pose = 0
    shoot_cfg  = 0
    while True:
      chessboard_Z = random.uniform( dist_from_camera[0], dist_from_camera[1] )
      chessboard_X = ( x - projection_matrix[0, 2] ) / projection_matrix[0, 0] * chessboard_Z 
      chessboard_Y = ( y - projection_matrix[1, 2] ) / projection_matrix[1, 1] * chessboard_Z 
      chessboard_position = np.matrix([ chessboard_X, chessboard_Y, chessboard_Z ])

      q = Quaternion().fromRPY( random.uniform( -pi/12., pi/12. ), random.uniform( -pi/12., pi/12. ), random.uniform( -pi/12., pi/12. ) ) 
      shoot_pose += 1
      R = q.toRotationMatrix()
      if (R * chessboard_normal)[2] >= 0.:
        continue

      Rt = np.hstack(( R, ( chessboard_position - camera_position ).transpose() ))
      
      if not all( [ isInImage( projectPoint( Rt, np.matrix(pt).transpose() ) ) for pt in chessboard_pts ] ):
        continue
        
      q = Quaternion().fromRPY(-pi, 0, -pi) * q  # Switch tn the real camera frame
      
      chessboard_pose = (chessboard_position[0,0], chessboard_position[0,1], chessboard_position[0,2]) + q.toTuple()
      ps.createTransformationConstraint ("gaze", "talos/rgbd_rgb_optical_joint", "mire/root_joint", chessboard_pose, [True,]*6)

      ps.resetConstraints ()
      ps.setNumericalConstraints('proj', foot_placement + ["com_talos_mire", "talos/left_gripper grasps mire/left", "gaze", ])
      ps.setLockedJointConstraints('proj', left_gripper_lock + right_gripper_lock + other_lock)
      
      res, qproj, err = ps.applyConstraints(qrand)
      if res:
        valid, msg = robot.isConfigValid (qproj)
        if valid:
            print "Found pose", shoot_pose, '\t', shoot_cfg
            expected_poses.append(chessboard_pose)
            q_proj_list.append(qproj)
            qrand = qproj[:]
            break
      # It may be needed to shoot from time to time but so far it works without doing it.
      #qrand = robot.shootRandomConfig()
      #shoot_cfg += 1
  ys.reverse()

hpp_poses = []
for q in q_proj_list:
    robot.setCurrentConfig(q)
    oMc = Transform(robot.getJointPosition('talos/rgbd_rgb_optical_joint'))
    oMm = Transform(robot.getJointPosition('mire/root_joint'))
    cMm = oMc.inverse() * oMm
    hpp_poses.append (cMm.toTuple())

res, hs_proj, err = graph.applyNodeConstraints("talos/left_gripper grasps mire/left", half_sitting)

paths = list()
failed = False
for i, (q1, q2) in enumerate(zip([hs_proj,] + q_proj_list, q_proj_list + [hs_proj,])):
    res, pid, msg = ps.directPath (q1, q2, True)
    if res:
        print "Path from", i, "to", i+1, ":", pid
        paths.append(pid)
    else:
        print "Could not joint", i, "to", i+1, ":", msg
        failed = True

ps.setParameter("SimpleTimeParameterization/safety", 0.5)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter('SimpleTimeParameterization/maxAcceleration', 1.)

cleanPaths = True
joinPaths = False

def displayPaths (viewer, paths):
    from hpp.gepetto import PathPlayer
    pp = PathPlayer (viewer, client=ps.client.basic)
    for p in paths:
        pp (p)

if not failed:
    if joinPaths:
        # join path
        i0 = paths[0]
        for i in paths[1:]:
            ps.concatenatePath (i0, i)

        if cleanPaths:
            for k,i in enumerate(paths[1:]):
                ps.erasePath (i-k)

        ps.optimizePath (i0)
        print 'Optimized path:', ps.numberPaths() - 1, ',', ps.pathLength(ps.numberPaths()-1)

    else:
        optpaths = []
        for i in paths:
            ps.optimizePath (i)
            optpaths.append (ps.numberPaths()-1)

        print 'Solution paths are\noptpaths=', str(optpaths)
        print 'displayPaths(v,optpaths) # to visualize paths'
