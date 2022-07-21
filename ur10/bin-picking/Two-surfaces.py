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

from http.client import REQUESTED_RANGE_NOT_SATISFIABLE
import sys, argparse, numpy as np, time, rospy
from turtle import forward
from math import pi, sqrt
from hpp import Transform
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import Robot, \
    createContext, newProblem, ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Rule, Constraints, CorbaClient, SecurityMargins
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory
from tools_hpp import RosInterface, PathGenerator
#from manipulation import Ground, Bin


UseAprilTagPlank = False
useFOV = False

class PartPlaque:
    urdfFilename = "package://agimus_demos/urdf/plaque-tubes-with-table.urdf"
    srdfFilename = "package://agimus_demos/srdf/plaque-tubes.srdf"
    rootJointType = "freeflyer"

class AprilTagPlank:
    urdfFilename = "package://agimus_demos/urdf/april-tag-plank.urdf"
    srdfFilename = "package://agimus_demos/srdf/april-tag-plank.srdf"
    rootJointType = "freeflyer"

#class Bin:
 #   urdfFilename = "package://agimus_demos/urdf/bin.urdf"
  #  srdfFilename = "package://agimus_demos/srdf/bin.srdf"
  #  rootJointType = "freeflyer"
class Bin (object):
  rootJointType = 'freeflyer'
  packageName = 'agimus_demos'
  meshPackageName = 'agimus_demos'
  urdfName = 'bin-picking-part'
  urdfSuffix = ""
  srdfSuffix = ""

class Ground:
    urdfFilename = "package://agimus_demos/urdf/ground.urdf"
    srdfFilename = "package://agimus_demos/srdf/ground.srdf"
    rootJointType = "anchor"


class Box:
    urdfFilename = "package://agimus_demos/urdf/box.urdf"
    srdfFilename = "package://agimus_demos/srdf/box.srdf"
    rootJointType = "anchor"

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

try:
    import rospy
    Robot.urdfString = rospy.get_param('robot_description')
    print("reading URDF from ROS param")
except:
    print("reading generic URDF")
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro\
      ("package://agimus_demos/urdf/ur10_bin-picking_sim.urdf.xacro",
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
ps = ProblemSolver(robot)
p = robot.hppcorba.problem.getProblem()
cdistance = p.getDistance()
croadmap = ps.client.manipulation.problem.createRoadmap(cdistance, crobot)
print("Robot loaded")
robot.opticalFrame = 'camera_color_optical_frame'
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
vf.loadEnvironmentModel (Ground, 'ground')
vf.loadEnvironmentModel  (Box, 'box')

#vf.loadObjectModel (Bin, 'part')

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
if UseAprilTagPlank:
    Part = AprilTagPlank
else:
    Part = Bin
vf.loadObjectModel (Part, "part")
robot.setJointBounds('part/root_joint', [0.0, 1.1, -0.8 ,1, -0.05, 0.8])
print("Part loaded")


robot.client.manipulation.robot.insertRobotSRDFModel\
    ("ur10e", "package://agimus_demos/srdf/ur10_robot.srdf")

partPose = [1.3, 0, 0,0,0,-sqrt(2)/2,sqrt(2)/2]

## Define initial configuration
q0 = robot.getCurrentConfig()
print("q0",q0)
# set the joint match with real robot
q0[:6] = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]
r = robot.rankInConfiguration['part/root_joint']
if UseAprilTagPlank:
    # Initial configuration of AprilTagPlank
    q0[r:r+7] = [1.3, 0, 0, 0, 0, -1, 0]
else:
    q0[r:r+7] = partPose
#q3=q0[::]
#q3[2]=0.3
#ps.setInitialConfig (q0)
#ps.addGoalConfig (q3)
## Home configuration
q_home = [-3.415742983037262e-05, -1.5411089223674317, 2.7125137487994593, -1.5707269471934815, -3.141557280217306, 6.67572021484375e-06, 1.3, 0, 0, 0, 0, -0.7071067811865476, 0.7071067811865476]
## Calibration configuration: the part should be wholly visible
q_calib = [1.5707, -3, 2.5, -2.8, -1.57, 0.0, 1.3, 0.0, 0.0, 0.0, 0.0, -0.7071067811865476, 0.7071067811865476]
## PointCloud position : the part should fit the field of view
q_pointcloud = [1.4802236557006836, -1.7792146009257812, 2.4035003821002405, -0.9398099416545411, 1.5034907341003418, -3.1523403135882773, 1.2804980083956572, 0.11300105405990518, -0.031348192422114174, -0.008769144315009561, 0.004377057629846714, -0.7073469546030107, 0.7067985775935985]
q_pointcloud2 = [1.480271816253662, -1.4792188129820762, 2.403522316609518, -0.9397409719279786, 1.503467082977295, -3.152398173009054, 1.166362251685465, 0.18398354959470994, -0.040275835859414855, -0.011115686791169354, 0.00903223476857969, -0.701584375075136, 0.7124424361958492]
q_pc1 = [1.4802236557006836, -2.7792393169798792, 2.6035669485675257, 0.06014220296826167, 1.5035147666931152, -3.1523261705981653, 1.1430909403610665, 0.22893782415973665, -0.04789935128099243, -0.0033794390562739483, 0.008636413673842097, -0.6985826418521135, 0.7154692755481825]
q_pc2 = [1.480247974395752, -2.7792188129820765, 2.6035461584674278, -0.13980153024707037, 1.5035266876220703, -3.1523383299456995, 1.1138463304333714, 0.23051956151197342, -0.040682700437678854, -0.00938303410217683, 0.013303913345295534, -0.7001874363145009, 0.7137734364544993]



def norm(quaternion):
    return sqrt(sum([e*e for e in quaternion]))

gripper = 'ur10e/gripper'
## Create specific constraint for a given handle
#  Rotation is free along x axis.
#  Note that locked part should be added to loop edge.
def createFreeRxConstraintForHandle(handle):
    name = gripper + ' grasps ' + handle
    handleJoint, jMh = robot.getHandlePositionInJoint(handle)
    gripperJoint, jMg = robot.getGripperPositionInJoint(gripper)
    ps.client.basic.problem.createTransformationConstraint2\
        (name, gripperJoint, handleJoint, jMg, jMh,
         [True, True, True, False, True, True])
    # pregrasp
    shift = 0.13
    M = Transform(jMg)*Transform([shift,0,0,0,0,0,1])
    name = gripper + ' pregrasps ' + handle
    ps.client.basic.problem.createTransformationConstraint2\
        (name, gripperJoint, handleJoint, M.toTuple(), jMh,
         [True, True, True, False, True, True])

## Build constraint graph
def createConstraintGraph():
    all_handles = ps.getAvailable('handle')
    part_handles = list(filter(lambda x: x.startswith("part/"), all_handles))
    objContactSurfaces =[['part/bottom',]]
    envSurfaces=['box/box_surface','ground/surface']


    graph = ConstraintGraph(robot, 'graph2')
    #rules = [Rule ([""], [""], True)]
    factory = ConstraintGraphFactory(graph)
    factory.setGrippers(["ur10e/gripper",])
    factory.environmentContacts (envSurfaces)
    factory.setObjects(['part',], [part_handles],objContactSurfaces )
    #factory.setRules (rules)
    factory.generate()
    print('factory.handle',factory.handles)
    print('all_handles ',all_handles )
    print('part_handles',part_handles)



    n = norm([-0.576, -0.002, 0.025, 0.817])
    ps.createTransformationConstraint('look-at-part', 'part/base_link', 'ur10e/wrist_3_link',
                                    [-0.126, -0.611, 1.209, -0.576/n, -0.002/n, 0.025/n, 0.817/n],
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
    sm.setSecurityMarginBetween("ur10e", "part", 0.015)
    sm.setSecurityMarginBetween("ur10e", "ur10e", 0)
    sm.defaultMargin = 0.01
    sm.apply()
    graph.initialize()
    # Set weights of levelset edges to 0
    for e in graph.edges.keys():
        if e[-3:] == "_ls" and graph.getWeight(e) != -1:
            graph.setWeight(e, 0)
    return graph


graph = createConstraintGraph()

try:
    v = vf.createViewer()
    #v(q0)
    pp = PathPlayer(v)
except:
    print("Did you launch the GUI?")

ri = None
ri = RosInterface(robot)
q_init = ri.getCurrentConfig(q0)
print("q_int",q_init)
# q_init = q0 #robot.getCurrentConfig()
pg = PathGenerator(ps, graph, ri, v, q_init)
pg.inStatePlanner.setEdge('Loop | f')
pg.testGraph()
NB_holes = 5 * 7
NB_holes_total = 44
hidden_holes = [2,10,11,12,14,16,24,33]  #remove 0
holes_to_do = [i for i in range(NB_holes) if i not in hidden_holes]
pg.setIsClogged(None)
ps.setTimeOutPathPlanning(10)
pg.setConfig("home", q_home)
pg.setConfig("calib", q_calib)
pg.setConfig("pointcloud", q_pointcloud)
pg.setConfig("pointcloud2", q_pointcloud2)
pg.setConfig("pointcloud_bas", q_pc1)
pg.setConfig("pointcloud_haut", q_pc2)
ps.resetGoalConfigs
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
    pg.setIsClogged(isClogged)
    visibleFeatures = lambda x : ur10_fov.visible(x, robot, featuress)

### DEMO

def getDoableHoles():
    doableHoles = []
    non_doableHoles = []
    pg.testGraph()
    for i in range(NB_holes_total):
        if pg.isHoleDoable(i):
            doableHoles.append(i)
        else:
            non_doableHoles.append(i)
    print("Doable holes : ", doableHoles)
    print("Non doable holes : ", non_doableHoles)
    return doableHoles, non_doableHoles

def doDemo():
    NB_holes_to_do = 7
    demo_holes = range(NB_holes_to_do)
    pids, qend = pg.planDeburringPaths(demo_holes)

holist = [7,8,9,42,43,13]
#v(q_init)

""" res2 ,res4 = False,False
while not (res2 and res4):
    q = robot.shootRandomConfig ()
    res1,q1,err = graph.applyNodeConstraints ('free', q)
    q = robot.shootRandomConfig ()
    res3,q2,err = graph.applyNodeConstraints ('free', q)
    if not res1 and res3:
        continue
    res2, msg = pg.robot.isConfigValid(q1)
    res4, msg = pg.robot.isConfigValid(q2)
ps.setInitialConfig (q1)
ps.addGoalConfig (q2)
v(q1) """




""" forward_tool = [[],[]]
for i in range(10000):
    res1,res = False,False
    while not res:
        qrand = robot.shootRandomConfig()
        res1, q6, error = graph.applyNodeConstraints ('ur10e/gripper > part/handle_00 | f_intersec', qrand)
        if not res1:
            continue
        res = crobot.setCurrentConfiguration(q6)
    forward_tool[0].append(q6[6])
    forward_tool[1].append(q6[7])
x_max = max(forward_tool[0])
x_min = min(forward_tool[0])
y_max = max(forward_tool[1])
y_min = min(forward_tool[1])   ##attation the area is rectangle so is not very percise """





import transforms3d
import pandas as pd
data = pd.read_csv('E0_0d.txt',header=None)
colo = data.shape[1]                            
row = data.shape[0]
empty = pd.DataFrame([0]*141).T
data = data.append(empty,ignore_index=True)   #remove empty '0' index
for i in range(data.shape[1]):
    mean  = data[i].mean()
    data.iloc[row,i] = mean

#take average from dataframe add to the last row

list = data.iloc[50].values.tolist()    # read as list

for i in range(len(list)):
    if list[i] == 0:
        del list[i:len(list)]      #remove rest value when meet 0
        break           #avoid index out of range

coor_part = [list[i:i+6] for i in range(0,len(list),6)]    #split each 6 ele

## convert from euler to quaternion
for i in range (len(coor_part)):
    rx = coor_part[i][3]
    ry = coor_part[i][4]
    rz = coor_part[i][5]
    quater =  transforms3d.euler.euler2quat(rx,ry ,rz ,axes='sxyz')
    coor_part[i][-3:] = quater


#Transform a given input pose of rigid body from one fixed frame to another
import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs  #    poseStamped  >> pose --header


def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame   #Frame this data is associated with
    pose_stamped.header.stamp = rospy.Time(0)    # return the latest available data for a specific transform

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


# Test Case
""" rospy.init_node("transform_test")

my_pose = Pose()
my_pose.position.x = -0.25
my_pose.position.y = -0.50
my_pose.position.z = +1.50
my_pose.orientation.x = 0.634277921154
my_pose.orientation.y = 0.597354098852
my_pose.orientation.z = 0.333048372508
my_pose.orientation.w = 0.360469667089

transformed_pose = transform_pose(my_pose, "wrist_3_link", "base_link") """


""" neutral_pose = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]
bin_des =  [0.6,0.5,0.00003,0,0,0,1]
for i in range (len(coor_part)):
    init_config =neutral_pose + coor_part[i]
    ps.setInitialConfig(init_config)
    goal_config = neutral_pose + bin_des
    ps.resetGoalConfigs()
    ps.addGoalConfig(goal_config)
    ps.solve() """





##test
""" final = [[0, -1.5707963267948966, 2.796017461694916, -1.5707963267948966, -3.141592653589793, 0.5, \
    0.5, 0.1, 3e-06, 0, 0, 0, 1],[0, -1.5707963267948966, 2.796017461694916, -1.5707963267948966, -3.141592653589793,\
         0.5, 0.5, 0.4, 3e-06, 0, 1, 0, 0],[0, -1.5707963267948966, 2.796017461694916, -1.5707963267948966, -3.141592653589793,\
         0.5, 0.5, 0, 3e-06, 0, 1, 0, 0]]    #substitue by coor_part
go = [0, -1.5707963267948966, 2.796017461694916, -1.5707963267948966, -3.141592653589793,\
            0.5, -0.4, 0.4, 3e-06, 0, 1, 0, 0]
res2 = ps.client.manipulation.problem.applyConstraints(graph.nodes['free'],go)
if not res2[0]:
    raise Exception ('Goal configuration could not be projected.')
go = res2[1]

ps.addPathOptimizer("RandomShortcut")
for i in range(len(final)):
    res1 = ps.client.manipulation.problem.applyConstraints(graph.nodes['free'],final[i])
    if not res1[0]:
        raise Exception ('Init configuration %s could not be projected.' % i)
        continue
    ini =  res1[1]
    ps.setInitialConfig(ini)
    ps.resetGoalConfigs()
    ps.addGoalConfig(go)
    ps.solve()
Nb_path = ps.numberPaths()
for i in range(1,Nb_path):
    if i%4== 0:
        ps.concatenatePath(0,i)  """




#test for box graph
"""
go =[2.444809329135643e-18, -1.5707963267948966, 2.796017461694916, -1.5707963267948966, -3.141592653589793, 0.5, 1.1, -0.02781554796245951, 0.09600101434050495, 1.3267600356985338e-06, 3.518114082643234e-06, 0.9999737254237396, -0.0072490308338324025]

res2 = ps.client.manipulation.problem.applyConstraints(graph.nodes['free_box'],go)

go = res2[1]


end = [2.775736622161967e-20, -1.5707963267948966, 2.796017461694916, -1.5707963267948966, -3.141592653589793, 0.5, 0.2, -0.45, 0.00095740333962464, 1.8704455211853133e-20, 3.253825530225867e-20, -0.2766468647918515, 0.9609716500505303]

res3 = ps.client.manipulation.problem.applyConstraints(graph.nodes['free'],end)
end = res3[1]

ps.setInitialConfig(go)
ps.resetGoalConfigs()
ps.addGoalConfig(end) """










[0, -1.5707963267948966, 2.796017461694916, -1.5707963267948966, -3.141592653589793, 0.5, 0.5, 0.4, 3e-06, 0, 1, 0, 0]