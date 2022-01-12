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
import random

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
ps.addPathOptimizer("SimpleShortcut")
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


## Define initial configuration
q0 = robot.getCurrentConfig()
q_home = [-3.415742983037262e-05, -1.5411089223674317, 2.7125137487994593, -1.5707269471934815, -3.141557280217306, 6.67572021484375e-06, 1.3, 0, 0, 0, 0, -0.7071067811865476, 0.7071067811865476]
# set the joint match with real robot
q0[:6] = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]
# q0[:3] = [0, -pi/2, pi/2]
r = robot.rankInConfiguration['part/root_joint']
# q0[r:r+7] = [0.0, -1.3, 0.8, 0, 0 ,1, 0]
q0[r:r+7] = [1.3, 0, 0,0,0,-sqrt(2)/2,sqrt(2)/2]
# Initial configuration of AprilTagPlank
# q0[r:r+7] = [1.3, 0, 0, 0, 0, -1, 0]


## Build constraint graph
all_handles = ps.getAvailable('handle')
part_handles = list(filter(lambda x: x.startswith("part/"), all_handles))

graph = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory(graph)
factory.setGrippers(["ur10e/gripper",])
factory.setObjects(["part",], [part_handles], [[]])
factory.generate()
sm = SecurityMargins(ps, factory, ["ur10e", "part"])
sm.setSecurityMarginBetween("ur10e", "part", 0.06)
sm.defaultMargin = 0.01
sm.apply()
graph.initialize()
# Set weights of levelset edges to 0
for e in graph.edges.keys():
    if e[-3:] == "_ls" and graph.getWeight(e) != -1:
        graph.setWeight(e, 0)


## Generate grasp configuration
ri = None

q_calib = q0[:]
q_calib[:13] = [1.5707,
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
# ri = RosInterface(robot)
# q_init = ri.getCurrentConfig(q0)
q_init = q0 #robot.getCurrentConfig()
from tools_hpp import PathGenerator
from hpp import Transform
pg = PathGenerator(ps, graph)
pg.inStatePlanner.setEdge('Loop | f')
holes_n = 5
holes_m = 7
NB_holes = holes_n * holes_m
NB_holes_total = 44
hidden_holes = [0,2,10,11,12,14,16,24,33]
holes_to_do = [i for i in range(NB_holes) if i not in hidden_holes]
isClogged = None
ps.setTimeOutPathPlanning(10)

useFOV = True
if useFOV:

    def configHPPtoFOV(q):
        return q[:6] + q[-7:]

    from ur10_fov import RobotFOV, RobotFOVGuiCallback, Feature, Features
    ur10_fov = RobotFOV(urdfString = Robot.urdfString,
                        fov = np.radians((69.4, 52)),
                        geoms = [],
                        optical_frame = "camera_color_optical_frame",
                        group_camera_link = "robot/ur10e/ref_camera_link",
                        camera_link = "ref_camera_link",
                        modelConfig = configHPPtoFOV)
    robot.setCurrentConfig(q_init)

    # Add Plaque model in the field of view object
    # to test if the plaque itself obstructs the view to the features
    oMh, oMd = robot.hppcorba.robot.getJointsPosition(q_init, ["universe", "part/base_link"])
    fMm = (Transform(oMh).inverse() * Transform(oMd)).toTuple()
    ur10_fov.appendUrdfModel(PartPlaque.urdfFilename, "universe",
        fMm, prefix="part/")

    feature_list = []
    for i in range(1, NB_holes_total+1):
        feature_list.append( Feature('part/hole_' + str(i).zfill(2) + '_link', 0.003) )
    featuress = [Features(feature_list, 2, 0.005, 0)]
    ur10_fov_gui = RobotFOVGuiCallback(robot, ur10_fov, featuress, modelConfig = configHPPtoFOV)
    # Display Robot Field of view.
    #vf.guiRequest.append( (ur10_fov.loadInGui, {'self':None}))
    # Display visibility cones.
    vf.addCallback(ur10_fov_gui)
    isClogged = lambda x : ur10_fov.clogged(x, robot, featuress)
    visibleFeatures = lambda x : ur10_fov.visible(x, robot, featuress)

try:
    v = vf.createViewer()
    v(q0)
    pp = PathPlayer(v)
except:
    print("Did you launch the GUI?")

def generatePath(index, qi):
    try:
        p = pg.generatePathForHandle('part/handle_'+str(index), qi, 50, isClogged=isClogged)
    except:
        print("Failure")
        return None, None
    if p is None:
        print("Failure")
        return None, None
    pid = ps.client.basic.problem.addPath(p)
    ps.optimizePath(pid)
    path_id = ps.numberPaths() - 1
    print("Path " + str(path_id))
    newq = ps.configAtParam(path_id, ps.pathLength(path_id))
    return path_id, newq

def vprint(verbose, msg):
    if verbose:
        print(msg)

def generatePointingPaths(concatenate=False, calib=True, verbose=False):
    ps.resetGoalConfigs()
    qi = q_init
    path_ids = []
    if calib:
        ps.setInitialConfig(q_init)
        ps.addGoalConfig(q_calib)
        ps.solve()
        ps.resetGoalConfigs()
        path_ids.append(ps.numberPaths()-1)
        vprint(verbose, "Path to calib config: " + str(path_ids[0]))
        qi = q_calib

    grasp_configs = []
    for index in holes_to_do:
        vprint(verbose,"Generating path for index " + str(index))
        path_id, newq = generatePath(index, qi)
        if newq is not None:
            qi = newq
            path_ids.append(path_id)
            grasp_configs.append(newq)
        else:
            vprint(verbose,"! FAILURE !")
            return False, path_ids, grasp_configs
    if concatenate:
        concat(path_ids)
    return True, path_ids, grasp_configs

def concat(path_ids):
    for i in path_ids[1:]:
        ps.concatenatePath(path_ids[0], i)
    return path_ids[0]

def visualize(path_ids):
    for index in path_ids:
        try:
            pp(index)
        except:
            print("Cannot play path. Is the GUI launched ?")

def go():
    res = False
    while not res:
        res, path_ids, config_grasps = generatePathPointing()
    return path_ids, config_grasps


generateTrajectory = False
if generateTrajectory:
    path_ids, grasp_configs = go()

def findConfigWithNVisibleFeatures(N=1, IterMax=1000):
    for i in range(IterMax):
        q = shootRandomValidConfig()
        n = visibleFeatures(q)[0]
        if n == N:
            print("Config", q, "sees", n, "features")
            return q

def shootRandomValidConfig(IterMax=100):
    for i in range(IterMax):
        q = robot.shootRandomConfig()
        r = robot.rankInConfiguration['part/root_joint']
        q[r:r+7] = [1.3, 0, 0,0,0,-sqrt(2)/2,sqrt(2)/2]
        if robot.isConfigValid(q)[0]:
            return q
    return None

def findGraspWithNVisibleFeatures(N=2, IterMax=10):
    for j in range(IterMax):
        to_do = holes_to_do[:]
        random.shuffle(to_do)
        for i in to_do:
            q = shootRandomValidConfig()
            res, qpg, qg = pg.generateValidConfigForHandle('part/handle_'+str(i), q_calib, [q], 50)
            if res:
                n = visibleFeatures(qg)[0]
                if n == N:
                    print("Foung grasp of hole", i, "with", N, "visible features")
                    return i, qg
    return None, None

oMh1, oMh2, oMh3 = robot.hppcorba.robot.getJointsPosition(q_init, ["part/hole_01_link", "part/hole_40_link", "part/hole_38_link"])
t1, t2, t3 = [np.array(m[:3]) for m in [oMh1, oMh2, oMh3] ]
n = np.cross(t2-t1, t3-t2)
n = n / np.linalg.norm(n) # unit normal vector of the part plane

def distanceBetweenCameraAndPart(q):
    translation_camera = np.array(robot.hppcorba.robot.getJointsPosition(q, ["ur10e/camera_color_optical_frame"])[0][:3])
    dist = abs(np.dot(n, translation_camera - t1))
    return dist

def solve(qi, qg):
    ps.setInitialConfig(qi)
    ps.resetGoalConfigs()
    ps.addGoalConfig(qg)
    ps.solve()
    return ps.numberPaths()-1

def generatePathNVisibleFeatures(N=None, IterMax=10):
    if N is None:
        N = [3]
    paths = []
    configs = [q_init, q_calib]
    p_id = solve(q_init, q_calib)
    paths.append(p_id)
    qi = q_calib
    for n in N:
        hole, q = findGraspWithNVisibleFeatures(N=n, IterMax=IterMax)
        if q is None:
            print("Failed to find grasp with", n, "visible features")
            return paths, configs
        ok, qpg, qg = pg.generateValidConfigForHandle('part/handle_'+str(hole), qi, [q])
        if not ok:
            print("Failure")
            return paths, configs
        configs.extend([qpg, qg, qpg])
        p1 = solve(qi, qpg)
        p2 = solve(qpg, qg)
        p3 = solve(qg, qpg)
        paths.extend([p1,p2,p3])
        qi = qpg[:]
    return paths, configs