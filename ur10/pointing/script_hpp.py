# Copyright 2021 CNRS - Airbus SAS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys, argparse, numpy as np, time, rospy
from math import pi, sqrt
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import Robot, loadServerPlugin, \
    createContext, newProblem, ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Rule, Constraints, CorbaClient, SecurityMargins
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory
from tools_hpp import RosInterface, concatenatePaths
from hpp.gepetto import PathPlayer

class PartPlaque:
    urdfFilename = "package://agimus_demos/urdf/plaque-tubes-with-table.urdf"
    srdfFilename = "package://agimus_demos/srdf/plaque-tubes.srdf"
    rootJointType = "freeflyer"

class AprilTagPlank:
    urdfFilename = "package://agimus_demos/urdf/april-tag-plank.urdf"
    srdfFilename = "package://agimus_demos/srdf/april-tag-plank.srdf"
    rootJointType = "freeflyer"
 
# parse arguments
defaultContext = "corbaserver"
p = argparse.ArgumentParser (description=
                             'Initialize demo of UR10 pointing')
p.add_argument ('--context', type=str, metavar='context',
                default=defaultContext,
                help="identifier of ProblemSolver instance")
args = p.parse_args ()

joint_bounds = {}
def setRobotJointBounds(which):
    for jn, bound in jointBounds[which]:
        robot.setJointBounds(jn, bound)

def planMotionToHole(ri, q0, handle):
    if ri is None:
        ri = RosInterface(robot)
    q_init = ri.getCurrentConfig(q0)
    c = ConfigGenerator(graph)
    res, qpg, qg = c.generateValidConfigForHandle(handle, q_init,
                                                  qguesses = [q_init,])
    if not res:
        raise RuntimeError("Failed to generate valid config for hole {}".
                           format(handle))
    ps.setInitialConfig(q_init)
    ps.addGoalConfig(qpg)
    ps.solve()

try:
    import rospy
    Robot.urdfString = rospy.get_param('robot_description')
    print("reading URDF from ROS param")
except:
    print("reading generic URDF")
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro\
      ("package://agimus_demos/urdf/ur10_robot_sim.urdf.xacro",
       "transmission_hw_interface:=hardware_interface/PositionJointInterface")
Robot.srdfString = ""

loadServerPlugin (args.context, "manipulation-corba.so")
newProblem()
client = CorbaClient(context=args.context)
#client.basic._tools.deleteAllServants()
client.manipulation.problem.selectProblem (args.context)
def wd(o):
    from hpp.corbaserver import wrap_delete
    return wrap_delete(o, client.basic._tools)

robot = Robot("robot", "ur10e", rootJointType="anchor", client=client)
crobot = wd(wd(robot.hppcorba.problem.getProblem()).robot())

print("Robot loaded")
robot.opticalFrame = 'camera_color_optical_frame'
ps = ProblemSolver(robot)
ps.loadPlugin("manipulation-spline-gradient-based.so")
ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
# ps.selectConfigurationShooter('Gaussian')
# ps.setParameter('ConfigurationShooter/Gaussian/center', 12*[0.] + [1.])
# ps.setParameter('ConfigurationShooter/Gaussian/standardDeviation', 0.25)
ps.setParameter('SimpleTimeParameterization/order', 2)
ps.setParameter('SimpleTimeParameterization/maxAcceleration', .5)
ps.setParameter('SimpleTimeParameterization/safety', 0.95)

# Add path projector to avoid discontinuities
ps.selectPathProjector ("Progressive", .05)
ps.selectPathValidation("Graph-Progressive", 0.01)
vf = ViewerFactory(ps)

## Shrink joint bounds of UR-10
#
jointBounds = dict()
jointBounds["default"] = [ (jn, robot.getJointBounds(jn)) \
                           if not jn.startswith('ur10/') else
                           (jn, [-pi, pi]) for jn in robot.jointNames]
jointBounds["limited"] = [('ur10e/shoulder_pan_joint', [-pi, pi]),
  ('ur10e/shoulder_lift_joint', [-pi, pi]),
  ('ur10e/elbow_joint', [-3.1, 3.1]),
  ('ur10e/wrist_1_joint', [-3.2, 3.2]),
  ('ur10e/wrist_2_joint', [-3.2, 3.2]),
  ('ur10e/wrist_3_joint', [-3.2, 3.2])]
setRobotJointBounds("limited")
## Remove some collision pairs
#
ur10JointNames = list(filter(lambda j: j.startswith("ur10/"), robot.jointNames))
ur10LinkNames = [ robot.getLinkNames(j) for j in ur10JointNames ]

## Load P72
#[1., 0, 0.8,0,0,-sqrt(2)/2,sqrt(2)/2]
Part = PartPlaque
# Part = AprilTagPlank

vf.loadRobotModel (Part, "part")
robot.setJointBounds('part/root_joint', [-2, 2, -2, 2, -2, 2])
print("Part loaded")

robot.client.manipulation.robot.insertRobotSRDFModel\
    ("ur10e", "package://agimus_demos/srdf/ur10_robot.srdf")

partPose = [1.3, 0, 0,0,0,-sqrt(2)/2,sqrt(2)/2]

## Define initial configuration
q0 = robot.getCurrentConfig()
q_home = [-3.415742983037262e-05, -1.5411089223674317, 2.7125137487994593, -1.5707269471934815, -3.141557280217306, 6.67572021484375e-06, 1.3, 0, 0, 0, 0, -0.7071067811865476, 0.7071067811865476]
# set the joint match with real robot
q0[:6] = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]
# q0[:3] = [0, -pi/2, pi/2]
r = robot.rankInConfiguration['part/root_joint']
# q0[r:r+7] = [0.0, -1.3, 0.8, 0, 0 ,1, 0]
q0[r:r+7] = partPose
# Initial configuration of AprilTagPlank
# q0[r:r+7] = [1.3, 0, 0, 0, 0, -1, 0]

## Build constraint graph
all_handles = ps.getAvailable('handle')
part_handles = list(filter(lambda x: x.startswith("part/"), all_handles))

graph = ConstraintGraph(robot, 'graph2')
factory = ConstraintGraphFactory(graph)
factory.setGrippers(["ur10e/gripper",])
factory.setObjects(["part",], [part_handles], [[]])
factory.generate()

import math
def norm(quaternion):
    norm_squared = 0
    for e in quaternion:
        norm_squared += e*e
    return math.sqrt(norm_squared)

n = norm([-0.599, -0.026, -0.069, 0.797])
ps.createTransformationConstraint('look-at-part', 'part/base_link', 'ur10e/wrist_3_link',
                                  [-0.012, -0.541, 1.285, -0.599/n, -0.026/n, -0.069/n, 0.797/n],
                                  [True, True, True, True, True, True,])
graph.createNode(['look-at-part'])
graph.createEdge('free', 'look-at-part', 'go-look-at-part', 1, 'free')
graph.createEdge('look-at-part', 'free', 'stop-looking-at-part', 1, 'free')

graph.addConstraints(node='look-at-part',
                     constraints = Constraints(numConstraints=['look-at-part']))

ps.createTransformationConstraint('placement/complement', '','part/base_link',
                                  [0,0,0,0, 0, 0, 1],
                                  [True, True, True, True, True, True,])
ps.setConstantRightHandSide('placement/complement', False)
graph.addConstraints(edge='go-look-at-part',
                     constraints = Constraints(numConstraints=\
                                               ['placement/complement']))
graph.addConstraints(edge='stop-looking-at-part',
                     constraints = Constraints(numConstraints=\
                                               ['placement/complement']))

sm = SecurityMargins(ps, factory, ["ur10e", "part"])
sm.setSecurityMarginBetween("ur10e", "part", 0.01)
sm.setSecurityMarginBetween("ur10e", "ur10e", 0)
sm.defaultMargin = 0.01
sm.apply()
graph.initialize()
# Set weights of levelset edges to 0
for e in graph.edges.keys():
    if e[-3:] == "_ls" and graph.getWeight(e) != -1:
        graph.setWeight(e, 0)


## Calibration configuration: the part should be wholly visible
q_calib = [1.5707,
 -3,
 2.5,
 -2.8,
 -1.57,
 0.0,
 1.3,
 0.0,
 0.0,
 0.0,
 0.0,
 -0.7071067811865476,
 0.7071067811865476]

from tools_hpp import RosInterface
ri = RosInterface(robot)
q_init = ri.getCurrentConfig(q0)
# q_init = robot.getCurrentConfig()
from tools_hpp import PathGenerator
pg = PathGenerator(ps, graph, ri, q_init)
pg.inStatePlanner.setEdge('Loop | f')
holes_n = 5
holes_m = 7
NB_holes = holes_n * holes_m
NB_holes_total = 44
hidden_holes = [0,2,10,11,12,14,16,24,33]
holes_to_do = [i for i in range(NB_holes) if i not in hidden_holes]
isClogged = None

useFOV = False
if useFOV:
    from ur10_fov import RobotFOV, RobotFOVGuiCallback, Feature, Features
    ur10_fov = RobotFOV(urdfString = Robot.urdfString,
                        fov = np.radians((69.4, 52)),
                        geoms = [],
                        optical_frame = "camera_color_optical_frame",
                        group_camera_link = "robot/ur10e/ref_camera_link",
                        camera_link = "ref_camera_link",
                        modelConfig = lambda q : q[:6])
    feature_list = []
    for i in range(1, NB_holes_total+1):
        feature_list.append( Feature('part/hole_' + str(i).zfill(2) + '_link', 0.003+0.001) )
    featuress = [Features(feature_list, 2, 0.01, 0.1)]
    ur10_fov_gui = RobotFOVGuiCallback(robot, ur10_fov, featuress, modelConfig = lambda q : q[:6])
    # Display Robot Field of view.
    #vf.guiRequest.append( (ur10_fov.loadInGui, {'self':None}))
    # Display visibility cones.
    vf.addCallback(ur10_fov_gui)
    isClogged = lambda x : ur10_fov.clogged(x, robot, featuress)

try:
    v = vf.createViewer()
    v(q0)
    pp = PathPlayer(v)
except:
    print("Did you launch the GUI?")

def addPath(p, optimizePath=True):
    pid = robot.client.basic.problem.addPath(p)
    if optimizePath:
        ps.optimizePath(pid)
    return ps.numberPaths()-1

def goToCalib():
    p = pg.goTo(q_calib)
    pid = addPath(p)
    return pid

def generateLocalizationConfig(qinit, maxIter=1000):
    for _ in range(maxIter):
        q = robot.shootRandomConfig()
        res, q, e = graph.generateTargetConfig('go-look-at-part', qinit, q)
        if res:
            return q
    raise RuntimeError("Failed to generate target config for localization")

def goToLocalization(qinit=q_init, maxIter=100):
    q_local = generateLocalizationConfig(qinit, maxIter=maxIter)
    p = pg.goTo(q_local)
    pid = addPath(p)
    return pid

def localizePart(qinit=q_init):
    q1 = ri.getCurrentConfig(qinit)
    new_q_init = ri.getObjectPose(q1)
    new_q_init[-4:] = [e/norm(new_q_init[-4:]) for e in new_q_init[-4:]]
    #v(new_q_init)
    return new_q_init

def planDeburringPathForHole(hole_id, qinit):
    p = pg.generatePathForHandle('part/handle_'+str(hole_id), qinit, 50, isClogged=isClogged)
    if p:
        pid = addPath(p)
    return pid

def planDeburringPaths(holes, qinit):
    qi = ri.getCurrentConfig(qinit)
    path_ids = []
    for hole_id in holes:
        print(qi)
        pid = planDeburringPathForHole(hole_id, qi)
        qi = ps.configAtParam(pid, ps.pathLength(pid))
        path_ids.append(pid)
    return path_ids

from scipy.spatial.transform import Rotation as R

# Create transformation constraint that will be used for generating
# configurations behind configurations after failure
# (useful if the robot fails in grasp position)
ps.client.basic.problem.createTransformationR3xSO3Constraint('behind-failure', '', 'ur10e/wrist_3_link',
                                  [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1],
                                  [True, True, True, True, True, True,])
ps.setConstantRightHandSide('behind-failure', False)
ps.addNumericalConstraints('config-projector', ['behind-failure'])

def moveBackwardsAfterFailure(qinit=q_init):
    qi = ri.getCurrentConfig(qinit)
    # Get current transform of wrist
    t = robot.getCurrentTransformation('ur10e/wrist_3_joint')
    trans = [t[i][3] for i in range(3)]
    rot_matrix = [t[i][:3] for i in range(3)]
    quat = list(R.from_matrix(rot_matrix).as_quat())
    # Set back 10cm
    wrist_x = np.matmul(np.array(rot_matrix), np.array([0,0,1]))
    new_trans = list(np.array(trans) - 0.1 * wrist_x)
    # Set the right hand side of the existing 'behind-failure' constraint
    ps.setRightHandSideByName('behind-failure', new_trans+quat)
    res, qgoal, err = ps.applyConstraints(qi)
    if not res:
        raise RuntimeError("Failed to project configuration")
    p = pg.goTo(qgoal, qinit=qi)
    pid = addPath(p)
    return pid

def eraseAllPaths(excepted=[]):
    for i in range(ps.numberPaths()-1,-1,-1):
        if i not in excepted:
            ps.erasePath(i)
