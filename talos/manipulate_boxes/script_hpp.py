# Copyright 2018, 2019, 2020 CNRS - Airbus SAS
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
from hpp import Quaternion, Transform
from hpp.corbaserver.manipulation import Constraints, ProblemSolver
from hpp.corbaserver.manipulation.robot import CorbaClient
from hpp.corbaserver import loadServerPlugin, createContext

from common_hpp import *

# parse arguments
p = argparse.ArgumentParser (description=
                             'Initialize demo of Pyrene manipulating a box')
p.add_argument ('--context', type=str, metavar='context',
                default=defaultContext,
                help="identifier of ProblemSolver instance")
args = p.parse_args ()
if args.context != defaultContext:
    createContext (args.context)

print("context=" + args.context)
loadServerPlugin (args.context, "manipulation-corba.so")

isSimulation = args.context == "simulation"

footPlacement = not isSimulation
comConstraint = not isSimulation
constantWaistYaw = not isSimulation
fixedArmWhenGrasping = not isSimulation

client = CorbaClient(context=args.context)
if args.context != defaultContext:
    client.manipulation.problem.selectProblem (args.context)

client.manipulation.problem.resetProblem()

robot, ps, vf, table, objects = makeRobotProblemAndViewerFactory(client, rolling_table=True)
if isSimulation:
    ps.setMaxIterProjection (1)


q_neutral = robot.getCurrentConfig()

# Set robot to neutral configuration before building constraint graph
robot.setCurrentConfig(q_neutral)

# create locked joint for table

com_constraint, foot_placement, foot_placement_complement = \
    createQuasiStaticEquilibriumConstraint (ps, init_conf)
gaze_constraint = createGazeConstraint (ps)
gaze_cost = createGazeCost (ps)
waist_constraint = createWaistYawConstraint (ps)
left_arm_lock  = createLeftArmLockedJoints (ps)
right_arm_lock = createRightArmLockedJoints (ps)
left_gripper_lock, right_gripper_lock = \
    createGripperLockedJoints (ps, init_conf)
table_lock = createTableLockedJoint (ps, table, init_conf)

graph = makeGraph(robot, table, objects)

# Add other locked joints in the edges.
for edgename, edgeid in graph.edges.items():
    graph.addConstraints(
        edge=edgename, constraints=Constraints(numConstraints=table_lock)
    )
# Add gaze and and COM constraints to each node of the graph
if comConstraint:
    for nodename, nodeid in graph.nodes.items():
        graph.addConstraints(
            node=nodename, constraints=Constraints(numConstraints=\
                com_constraint + gaze_constraint
            )
        )

# Add locked joints and foot placement constraints in the graph,
# add foot placement complement in each edge.
if footPlacement:
    for edgename, edgeid in graph.edges.items():
        graph.addConstraints(
            edge=edgename,
            constraints=Constraints(numConstraints=foot_placement_complement),
        )

if constantWaistYaw:
    for edgename, edgeid in graph.edges.items():
        graph.addConstraints(
            edge=edgename, constraints=Constraints(
                numConstraints=waist_constraint
            )
        )

graph.addConstraints(
    graph=True,
    constraints=Constraints(
        numConstraints=foot_placement + left_gripper_lock + right_gripper_lock,
    ),
)

# On the real robot, the initial configuration as measured by sensors is very
# likely not in any state of the graph. State "starting_state" and transition
# "starting_motion" are aimed at coping with this issue.
graph.createNode("starting_state")
graph.createEdge("starting_state", "free", "starting_motion", isInNode="starting_state")
graph.createEdge(
    "starting_state", "starting_state", "loop_ss", isInNode="starting_state", weight=0
)
graph.createEdge(
    "free",
    "starting_state",
    "go_to_starting_state",
    isInNode="starting_state",
    weight=0,
)
graph.addConstraints(
    node="starting_state", constraints=Constraints(numConstraints=["place_box"])
)
graph.addConstraints(
    edge="loop_ss",
    constraints=Constraints(
        numConstraints=["place_box/complement"] + table_lock
    ),
)
graph.addConstraints(
    edge="starting_motion",
    constraints=Constraints(
        numConstraints=["place_box/complement"] + table_lock
    ),
)
graph.addConstraints(
    edge="go_to_starting_state",
    constraints=Constraints(
        numConstraints=["place_box/complement"] + table_lock
    ),
)

# Loop transitions
e_l_l1 = "Loop | 0-0"
e_l_r1 = "Loop | 1-0"
e_l_l2 = "Loop | 0-1"
e_l_r2 = "Loop | 1-1"
e_l_l3 = "Loop | 0-2"
e_l_r3 = "Loop | 1-2"
e_l_l4 = "Loop | 0-3"
e_l_r4 = "Loop | 1-3"
# Transitions from state 'free'
e_l1 = "talos/left_gripper > box/handle1 | f"
e_r1 = "talos/right_gripper > box/handle1 | f"
e_l2 = "talos/left_gripper > box/handle2 | f"
e_r2 = "talos/right_gripper > box/handle2 | f"
e_l3 = "talos/left_gripper > box/handle3 | f"
e_r3 = "talos/right_gripper > box/handle3 | f"
e_l4 = "talos/left_gripper > box/handle4 | f"
e_r4 = "talos/right_gripper > box/handle4 | f"
# Transitions from one grasp to two grasps
e_l1_r2 = "talos/right_gripper > box/handle2 | 0-0"
e_l1_r4 = "talos/right_gripper > box/handle4 | 0-0"
e_r1_l2 = "talos/left_gripper > box/handle2 | 1-0"
e_r1_l4 = "talos/left_gripper > box/handle4 | 1-0"
e_l2_r1 = "talos/right_gripper > box/handle1 | 0-3"
e_l2_r3 = "talos/right_gripper > box/handle3 | 0-3"
e_r2_l1 = "talos/left_gripper > box/handle3 | 1-3"
e_r2_l3 = "talos/left_gripper > box/handle3 | 1-3"
e_r3_l4 = "talos/left_gripper > box/handle4 | 1-2"
e_r3_l2 = "talos/left_gripper > box/handle2 | 1-2"
e_l3_r4 = "talos/right_gripper > box/handle4 | 0-2"
e_l3_r2 = "talos/right_gripper > box/handle2 | 0-2"
e_l4_r1 = "talos/right_gripper > box/handle1 | 0-3"
e_l4_r3 = "talos/right_gripper > box/handle3 | 0-3"
e_r4_l1 = "talos/left_gripper > box/handle1 | 1-3"
e_r4_l3 = "talos/left_gripper > box/handle3 | 1-3"
# Transition from 'free' to first waypoint
e_l1_app = e_l1 + "_01"
e_r1_app = e_r1 + "_01"
e_l2_app = e_l2 + "_01"
e_r2_app = e_r2 + "_01"
e_l3_app = e_l3 + "_01"
e_r3_app = e_r3 + "_01"
e_l4_app = e_l4 + "_01"
e_r4_app = e_r4 + "_01"

if fixedArmWhenGrasping:
    leftArmConstraint = Constraints(numConstraints=left_arm_lock)
    rightArmConstraint = Constraints(numConstraints=right_arm_lock)

    graph.addConstraints(edge=e_l1_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r1_app, constraints=leftArmConstraint)
    graph.addConstraints(edge=e_l2_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r2_app, constraints=leftArmConstraint)
    graph.addConstraints(edge=e_l3_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r3_app, constraints=leftArmConstraint)
    graph.addConstraints(edge=e_l4_app, constraints=rightArmConstraint)
    graph.addConstraints(edge=e_r4_app, constraints=leftArmConstraint)
    graph.initialize()


ps.setRandomSeed(123)
ps.selectPathProjector("Progressive", 0.2)
# ps.selectPathValidation("Progressive", 0.01)
ps.selectPathValidation("Discretized", 0.01)
# ps.selectPathValidation("Dichotomy", 0.0)
graph.setWeight ('Loop | f', 1)

graph.initialize()
for edge in [ e_l1_r2, e_l1_r4, e_r1_l2, e_r1_l4, e_l2_r1, e_l2_r3, e_r2_l1, e_r2_l3, e_r3_l4, e_r3_l2, e_l3_r4, e_l3_r2, e_l4_r1, e_l4_r3, e_r4_l1, e_r4_l3,]:
    addCostToComponent(graph, gaze_cost, edge=edge+"_01")

q_init = [
    0.5402763680625408,
    -0.833196863501999,
    1.0199316910041052,
    -0.03128842007165536,
    0.013789190720970665,
    0.7297271046306221,
    0.6828830395873728,
    -0.002517657851415276,
    0.03462520266527989,
    -0.5316498053579248,
    0.8402250533557625,
    -0.3730641123290547,
    -0.011780954381969872,
    -0.0025270209724267243,
    0.034480300571697056,
    -0.5168007496652326,
    0.8113706150231745,
    -0.3590584062316795,
    -0.011635750462120158,
    0.0,
    0.4392076095335054,
    0.2806705510519144,
    0.5,
    0.0019674899062759165,
    -0.5194264855927397,
    1.2349417194832937e-05,
    0.0007850050683513623,
    0.10090925286890041,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -0.2814831804277627,
    -0.5,
    -0.004238959829568303,
    -0.5200522586579716,
    0.00014996678886283413,
    -0.0015425422291322729,
    0.10092910629223316,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.5143008291852817,
    0.0455661913503581,
    0.45891797741593393,
    -0.25,
    0.832,
    -0.5,
    0.5,
    0.5,
    0.5,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
]

q_init_2 = [
 0.10232996455082483,
 -0.6765907825598196,
 1.0127532891272562,
 -0.03128842007165536,
 0.013789190720970663,
 0.7297271046306221,
 0.6828830395873728,
 -0.06433916542939629,
 0.043870498049689115,
 -0.5628242161042075,
 0.8841899013297069,
 -0.38432624897999573,
 -0.018779273888610132,
 -0.0643874479810337,
 0.043105369681181664,
 -0.5358746332937586,
 0.8649692122276535,
 -0.39205304402506713,
 -0.018012626496066456,
 0.024791641046798815,
 0.4783976095641855,
 0.25329292829053024,
 0.49327117282521676,
 0.000516614757372389,
 -0.5245691832414258,
 -2.0805143073940602e-05,
 0.00033056419866755993,
 0.10017774309160198,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 -0.2561311712465022,
 -0.46825515018405794,
 0.0017989159336655495,
 -0.5246715517230384,
 1.3018926440700662e-05,
 0.00020463708980818305,
 0.10020429676688965,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.5257065303386299,
 0.06401831552114343,
 0.0002964373153980979,
 -0.2,
 0.8574835283167916,
 -0.5,
 0.5,
 0.5,
 0.5,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 1.0]

setGaussianShooter (ps, table, objects, q_init, 0.1)

# Set Optimization parameters
ps.setParameter("SimpleTimeParameterization/safety", 0.25)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)
ps.setParameter("ManipulationPlanner/extendStep", 0.7)
ps.setMaxIterPathPlanning(50)

if args.context == defaultContext:
    # Define problem
    res, q_init, err = graph.generateTargetConfig("Loop | f", q_init, q_init)
    if not res:
        raise RuntimeError("Failed to project initial configuration")

    q_goal = q_init[::]
    rank = robot.rankInConfiguration["box/root_joint"]
    q_goal[rank + 3 : rank + 7] = (
        Quaternion([0, 1, 0, 0]) * Quaternion(q_init[rank + 3 : rank + 7])).\
        toTuple()
    # res, q_goal, err = graph.applyNodeConstraints ('free', q_goal)
    res, q_proj, err = graph.generateTargetConfig("Loop | f", q_goal, q_goal)
    if not res:
        raise RuntimeError("Failed to project goal configuration")
    assert q_init[-7:] == q_goal[-7:]

    solver = Solver(
        ps,
        graph,
        q_init,
        q_goal,
        e_l_l1,
        e_l_r1,
        e_l_l2,
        e_l_r2,
        e_l_l3,
        e_l_r3,
        e_l_l4,
        e_l_r4,
        e_l1,
        e_r1,
        e_l2,
        e_r2,
        e_l3,
        e_r3,
        e_l4,
        e_r4,
        e_l1_r2,
        e_l1_r4,
        e_r1_l2,
        e_r1_l4,
        e_l2_r1,
        e_l2_r3,
        e_r2_l1,
        e_r2_l3,
        e_r3_l4,
        e_r3_l2,
        e_l3_r4,
        e_l3_r2,
        e_l4_r1,
        e_l4_r3,
        e_r4_l1,
        e_r4_l3,
    )

    qBoxVisible, pathId = solver.makeBoxVisibleFrom(init_conf, True, True)

# From an estimated configuration with position of objects
# solver.solveFromEstimatedConfiguration (init_conf)

## Solving with ManipulationRRT and random shortcut takes approximately 2 minutes
# ps.setMaxIterPathPlanning(1000)
# ps.clearPathOptimizers ()
# ps.addPathOptimizer ("RandomShortcut")
# ps.addPathOptimizer ("SimpleTimeParameterization")
# ps.setInitialConfig (q_init)
# ps.addGoalConfig (q_goal)
# time = ps.solve ()
