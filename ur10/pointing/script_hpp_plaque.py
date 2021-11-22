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
from hpp.gepetto.manipulation import ViewerFactory
from tools_hpp import ConfigGenerator, RosInterface, concatenatePaths

class PartPlaque:
    urdfFilename = "package://agimus_demos/urdf/plaque-tubes-with-table.urdf"
    srdfFilename = "package://agimus_demos/srdf/plaque-tubes.srdf"
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

robot = Robot("robot", "ur10e", rootJointType="anchor", client=client)
print("Robot loaded")
robot.opticalFrame = 'xtion_rgb_optical_frame'
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
ps.selectPathValidation("Dichotomy", 0)
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
ur10JointNames = filter(lambda j: j.startswith("ur10/"), robot.jointNames)
ur10LinkNames = [ robot.getLinkNames(j) for j in ur10JointNames ]

## Load P72
#[1., 0, 0.8,0,0,-sqrt(2)/2,sqrt(2)/2]
vf.loadRobotModel (PartPlaque, "part")
robot.setJointBounds('part/root_joint', [-2, 2, -2, 2, -2, 2])
print("PartPlaque loaded")

robot.client.manipulation.robot.insertRobotSRDFModel\
    ("ur10e", "package://agimus_demos/srdf/ur10_robot.srdf")


## Define initial configuration
q0 = robot.getCurrentConfig()
# set the joint match with real robot
q0[:6] = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.0]
# q0[:3] = [0, -pi/2, pi/2]
r = robot.rankInConfiguration['part/root_joint']
# q0[r:r+7] = [0.0, -1.3, 0.8, 0, 0 ,1, 0]
q0[r:r+7] = [1.3, 0, 0,0,0,-sqrt(2)/2,sqrt(2)/2]

## Build constraint graph
all_handles = ps.getAvailable('handle')
part_handles = filter(lambda x: x.startswith("part/"), all_handles)

graph = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory(graph)
factory.setGrippers(["ur10e/gripper",])
factory.setObjects(["part",], [part_handles], [[]])
factory.generate()
sm = SecurityMargins(ps, factory, ["ur10e", "part"])
sm.setSecurityMarginBetween("ur10e", "ur10e", 0.02)
sm.defaultMargin = 0.01
sm.apply()
graph.initialize()
# Set weights of levelset edges to 0
for e in graph.edges.keys():
    if e[-3:] == "_ls" and graph.getWeight(e) != -1:
        graph.setWeight(e, 0)


## Generate grasp configuration
ri = None

try:
    v = vf.createViewer()
    v(q0)
except:
    print("Did you launch the GUI?")


q_calib = [1.5707,
 -3,
 2.5,
 -2.5,
 -1.57,
 0.0,
 1.3,
 0.0,
 0.0,
 0.0,
 0.0,
 -0.7071067811865476,
 0.7071067811865476]


generateTrajectory = True
if generateTrajectory:
    from tools_hpp import RosInterface
    ri = RosInterface(robot)
    q_init = ri.getCurrentConfig(q0)
    # q_init = robot.getCurrentConfig()
    from tools_hpp import PathGenerator
    from hpp.gepetto import PathPlayer
    pg = PathGenerator(ps, graph)
    pp = PathPlayer(v)
    pg.inStatePlanner.setEdge('Loop | f')
    holes_n = 5
    holes_m = 7
    NB_holes = holes_n * holes_m
    hidden_holes = [0,2,11,12,14,17,20,24,33]
    holes_to_do = [i for i in range(NB_holes) if i not in hidden_holes]

    def generatePath(index, qi):
        try:
            p = pg.generatePathForHandle('part/handle_'+str(index), qi, 200)
        except:
            print("Failure")
            return None, None
        ps.client.basic.problem.addPath(p)
        path_id = ps.numberPaths() - 1
        print("Path " + str(path_id))
        newq = ps.configAtParam(path_id, ps.pathLength(path_id))
        return path_id, newq

    def go(concatenate=False):
        ps.resetGoalConfigs()
        ps.setInitialConfig(q_init)
        ps.addGoalConfig(q_calib)
        ps.solve()
        ps.resetGoalConfigs()
        path_ids = [ps.numberPaths()-1]
        print("Path to calib config: " + str(path_ids[0]))

        qi = q_calib
        grasp_configs = []
        for index in holes_to_do:
            print("Generating path for index " + str(index))
            path_id, newq = generatePath(index, qi)
            if newq is not None:
                qi = newq
                path_ids.append(path_id)
                grasp_configs.append(newq)
        if concatenate:
            concat(path_ids)
        return path_ids, grasp_configs

    def concat(path_ids):
        for i in path_ids[1:]:
            ps.concatenatePath(path_ids[0], i)
        return path_ids[0]

    def visualize(path_ids):
        for index in path_ids:
            pp(index)

    path_ids, grasp_configs = go()