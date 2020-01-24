# Copyright 2020 CNRS - Airbus SAS
# Author: Florent Lamiraux, Joseph Mirabel, Alexis Nicolin
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

import argparse, numpy as np
from hpp import Transform
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import Constraints, ConstraintGraph, \
    ProblemSolver, Rule
from hpp.corbaserver.manipulation.robot import CorbaClient, HumanoidRobot
from hpp.gepetto.manipulation import ViewerFactory
from hpp.corbaserver.manipulation.constraint_graph_factory import \
    ConstraintGraphFactory

from common_hpp import createGripperLockedJoints, createLeftArmLockedJoints, \
    createRightArmLockedJoints, createQuasiStaticEquilibriumConstraint, \
    createWaistYawConstraint, defaultContext, shrinkJointRange

loadServerPlugin (defaultContext, "manipulation-corba.so")

client = CorbaClient(context=defaultContext)
client.manipulation.problem.resetProblem()

# parse arguments
p = argparse.ArgumentParser (description=
                             'Procuce motion to calibrate arms and camera')
p.add_argument ('--arm', type=str, metavar='arm',
                default='left',
                help="which arm: 'right' or 'left'")
p.add_argument ('--N', type=int, metavar='N', default=10,
                help="number of configurations generated")
args = p.parse_args ()

# distance between configurations
def distance (ps, q0, q1) :
    ''' Distance between two configurations of the box'''
    assert (len (q0) == ps.robot.getConfigSize ())
    assert (len (q1) == ps.robot.getConfigSize ())
    distance = ps.hppcorba.problem.getDistance ()
    return distance.call (q0, q1)

# Gaze constraint
def createGazeConstraint (ps, whichArm):
    ps.createPositionConstraint(
        "gaze",
        "talos/rgbd_optical_joint",
        "talos/arm_" + whichArm + "_7_joint",
        (0, 0, 0),
        (0, 0, -0.1),
        (True, True, False),
    )
    return ["gaze"]

# Check that target frame of gaze constraint is not behind the camera
def validateGazeConstraint (ps, q, whichArm):
    robot = ps.robot
    robot.setCurrentConfig (q)
    Mcamera = Transform (robot.getLinkPosition ("talos/rgbd_optical_frame"))
    Mtarget = Transform (robot.getLinkPosition ("talos/arm_" + whichArm +
                                                "_7_link"))
    z = (Mcamera.inverse () * Mtarget).translation [2]
    if z < .1: return False
    return True
    
HumanoidRobot.packageName = "talos_data"
HumanoidRobot.urdfName = "pyrene"
HumanoidRobot.urdfSuffix = ""
HumanoidRobot.srdfSuffix = ""
HumanoidRobot.leftAnkle = "talos/leg_left_6_joint"
HumanoidRobot.rightAnkle = "talos/leg_right_6_joint"

initConf = [0, 0, 1.095, 0, 0, 0, 1, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0, 0.006761, 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0]

robot = HumanoidRobot("talos", "talos", rootJointType="freeflyer")
shrinkJointRange (robot, 0.95)
# set freeflyerjoint bounds
robot.setJointBounds ("talos/root_joint",
                      [-0.5, 0.5,-0.5, 0.5,0.5, 1.5,
                       -1.01, 1.01, -1.01, 1.01, -1.01, 1.01, -1.01, 1.01])

ps = ProblemSolver (robot)
ps.setErrorThreshold (1e-4)
ps.setMaxIterProjection (40)

vf = ViewerFactory (ps)

left_arm_lock  = createLeftArmLockedJoints (ps)
right_arm_lock = createRightArmLockedJoints (ps)
arm_locked = right_arm_lock if args.arm == 'left' else left_arm_lock
left_gripper_lock, right_gripper_lock = createGripperLockedJoints (ps, initConf)
com_constraint, foot_placement, foot_placement_complement = \
    createQuasiStaticEquilibriumConstraint (ps, initConf)
gaze_constraint = createGazeConstraint (ps,args.arm)
waist_constraint = createWaistYawConstraint (ps)

graph = ConstraintGraph(robot, "graph")

graph.createNode(["free", "starting_state"])
graph.createEdge("starting_state", "free", "starting_motion", isInNode="starting_state")
graph.createEdge("free", "starting_state", "go_to_starting_state", isInNode="starting_state")
graph.createEdge("free", "free", "Loop | f", isInNode="starting_state")

# Set constraints
graph.addConstraints (node = "starting_state", constraints = Constraints (
    numConstraints = com_constraint + foot_placement + left_gripper_lock +
    right_gripper_lock
))
graph.addConstraints (node = "free", constraints = Constraints (
    numConstraints = com_constraint + foot_placement + left_gripper_lock +
    right_gripper_lock + gaze_constraint + waist_constraint
))
graph.addConstraints (edge = "Loop | f", constraints = Constraints (
    numConstraints = com_constraint + foot_placement + arm_locked
))

graph.initialize ()

ps.setParameter("SimpleTimeParameterization/safety", 0.5)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", .25)
ps.addPathOptimizer ("SimpleTimeParameterization")

res, q, err = graph.generateTargetConfig ("starting_motion", initConf,
                                          initConf)
if not res:
    raise RuntimeError ('Failed to project initial configuration')

ps.setInitialConfig (initConf)
ps.addGoalConfig (q)
ps.solve ()
assert (ps.numberPaths () == 2)
ps.erasePath (0)

configs = [q [::]]

# Generate N random configurations
N = args.N
i = 1
while i < N:
    q = robot.shootRandomConfig ()
    res, q1, err = graph.generateTargetConfig ("Loop | f", configs [0], q)
    if not res: continue
    res = validateGazeConstraint (ps, q1, args.arm)
    if not res: continue
    res, msg = robot.isConfigValid (q1)
    if res:
        configs.append (q1)
        i += 1

# Build matrix of distances between box poses
dist = np.matrix (np.zeros (N*N).reshape (N,N))
for i in range (N):
    for j in range (i+1,N):
        dist [i,j] = distance (ps, configs [i], configs [j])
        dist [j,i] = dist [i,j]

# order configurations according to naive solution to traveler salesman problem
notVisited = range (1,N)
visited = range (0,1)
while len (notVisited) > 0:
    # rank of current configuration in visited
    i = visited [-1]
    # find closest not visited configuration
    m = 1e20
    closest = None
    for j in notVisited:
        if dist [i,j] < m:
            m = dist [i,j]
            closest = j
    notVisited.remove (closest)
    visited.append (closest)
visited.append (len (visited))
configs.append (initConf)

print (visited)

for i0, i1 in zip (visited, visited [1:]):
    q_init = configs [i0]
    q_goal = configs [i1]
    ps.resetGoalConfigs ()
    ps.setInitialConfig (q_init)
    ps.addGoalConfig (q_goal)
    ps.solve ()
    pid = ps.numberPaths () - 2
    # remove non optimized path
    ps.erasePath (pid)
