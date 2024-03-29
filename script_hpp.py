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
from hpp import Transform
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import Robot, \
    createContext, newProblem, ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Rule, Constraints, CorbaClient, SecurityMargins
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory
from tools_hpp import RosInterface, PathGenerator

useFOV = False

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

ros = False
try:
    import rospy
    Robot.urdfString = rospy.get_param('robot_description')
    print("reading URDF from ROS param")
    ros = True
except:
    print("reading generic URDF")
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro\
      ("package://agimus_demos/ur10/pointing/urdf/robot.urdf.xacro",
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
# Bounds to generate calibration configurations
jointBounds["calibration"] = [('ur10e/shoulder_pan_joint', [-2.5, 2.5]),
  ('ur10e/shoulder_lift_joint', [-2.5, 2.5]),
  ('ur10e/elbow_joint', [-2.5, 2.5]),
  ('ur10e/wrist_1_joint', [-2.5, 2.5]),
  ('ur10e/wrist_2_joint', [-2.5, 2.5]),
  ('ur10e/wrist_3_joint', [-2.5, 2.5])]
setRobotJointBounds("limited")
## Remove some collision pairs
#
ur10JointNames = list(filter(lambda j: j.startswith("ur10/"), robot.jointNames))
ur10LinkNames = [ robot.getLinkNames(j) for j in ur10JointNames ]

class Demo():
    def __init__(self, yaml_data):
        if yaml_data is None:
            self.load_from_ros()
        else:
            self.load_from_yaml(yaml_data)
    def load_from_yaml(self, yaml_data):
        class Part:
            name = yaml_data['demo']['objects']['part']['class_name']
            urdfFilename = f"package://{yaml_data['demo']['objects']['part']['urdf']['package']}/{yaml_data['demo']['objects']['part']['urdf']['file']}"
            srdfFilename = f"package://{yaml_data['demo']['objects']['part']['srdf']['package']}/{yaml_data['demo']['objects']['part']['srdf']['file']}"
            rootJointType = yaml_data['demo']['objects']['part']['root_joint_type']
            pose = yaml_data['demo']['objects']['part']['pose']
            bounds = yaml_data['demo']['objects']['part']['bounds']
            configs = yaml_data['demo']['configurations']
        self.configs = yaml_data['demo']['configurations']
        self.Part = Part()
        
    def load_from_ros(self):
        class Part:
            name = rospy.get_param('/demo/objects/part/class_name')
            urdfFilename = f"package://{rospy.get_param('/demo/objects/part/urdf/package')}/{rospy.get_param('/demo/objects/part/urdf/file')}"
            srdfFilename = f"package://{rospy.get_param('/demo/objects/part/srdf/package')}/{rospy.get_param('/demo/objects/part/srdf/file')}"
            rootJointType = rospy.get_param('/demo/objects/part/root_joint_type')
            pose = rospy.get_param('/demo/objects/part/pose')
            bounds = rospy.get_param('/demo/objects/part/bounds')
        self.configs = rospy.get_param('/demo/configurations')
        self.Part = Part
    
if ros:
    print("Loading part")
    Demo = Demo(None)
    Part = Demo.Part
else:
    import yaml
    import glob
    yaml_files = glob.glob("demo*.yaml")
    print("Choose a yaml file:")
    for i, f in enumerate(yaml_files):
        print(f"{i}: {f}")
    yaml_file = yaml_files[int(input())]
    with open(yaml_file, 'r') as stream:
        try:
            demoData = yaml.safe_load(stream)
            Demo = Demo(demoData)
            Part = Demo.Part
        except yaml.YAMLError as exc:
            print(exc)

vf.loadRobotModel (Part, "part")
robot.setJointBounds('part/root_joint', Part.bounds)
print(f"{Part.name} loaded")

robot.client.manipulation.robot.insertRobotSRDFModel\
    ("ur10e", "package://agimus_demos/srdf/ur10_robot.srdf")
## Define initial configuration
q0 = robot.getCurrentConfig()
# set the joint match with real robot
q0[:6] = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]
r = robot.rankInConfiguration['part/root_joint']
q0[r:r+7] = Part.pose

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

    graph = ConstraintGraph(robot, 'graph2')
    factory = ConstraintGraphFactory(graph)
    factory.setGrippers(["ur10e/gripper",])
    factory.setObjects(["part",], [part_handles], [[]])
    factory.generate()
    for handle in all_handles:
        loopEdge = 'Loop | 0-{}'.format(factory.handles.index(handle))
        graph.addConstraints(edge = loopEdge, constraints = Constraints
            (numConstraints=['part/root_joint']))

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
    return factory, graph

factory, graph = createConstraintGraph()

try:
    v = vf.createViewer()
    v(q0)
    pp = PathPlayer(v)
except:
    print("Did you launch the GUI?")

ri = None
ri = RosInterface(robot)
q_init = ri.getCurrentConfig(q0)
# q_init = q0 #robot.getCurrentConfig()
pg = PathGenerator(ps, graph, ri, v, q_init)
pg.inStatePlanner.setEdge('Loop | f')
pg.testGraph()
NB_holes = 5 * 7
NB_holes_total = 44
hidden_holes = [0,2,10,11,12,14,16,24,33]
holes_to_do = [i for i in range(NB_holes) if i not in hidden_holes]
pg.setIsClogged(None)
ps.setTimeOutPathPlanning(10)

# loop through Demo.configs keys and set the config
for key,config in Demo.configs.items():
    pg.setConfig(key, config)

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
    pids, qend = pg.planPointingPaths(demo_holes)

holist = [7,8,9,42,43,13]
v(q_init)

def launchTeachWidget():
    from TeachWidget.MainWindow import MainWindow
    from PyQt5 import QtWidgets
    
    app = QtWidgets.QApplication([])
    window = MainWindow(pg=pg, ri=ri, robot=robot, ps=ps, qApplication= app)
    window.show()
    app.exec_()