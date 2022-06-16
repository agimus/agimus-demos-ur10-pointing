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

from datetime import datetime
from math import sqrt
import numpy as np
import json

from hpp import Quaternion
from hpp.corbaserver import wrap_delete, Client
from hpp.corbaserver.manipulation import ConstraintGraph, ProblemSolver, Rule, \
    SecurityMargins, Constraints
from hpp.corbaserver.manipulation.robot import HumanoidRobot
from hpp.gepetto.manipulation import ViewerFactory
from hpp.corbaserver.manipulation.constraint_graph_factory import \
    ConstraintGraphFactory

defaultContext = "corbaserver"
client = Client()

def wd(o):
    return wrap_delete(o, client._tools)

class Table(object):
    contacts = ["top"]
    handles = []
    rootJointType = "freeflyer"
    urdfFilename = "package://gerard_bauzil/urdf/table_140_70_73.urdf"
    srdfFilename = "package://gerard_bauzil/srdf/table_140_70_73.srdf"
    def __init__(self, name, vf):
        self.name = name
        vf.loadObjectModel(self.__class__, name)
        self.handles = [name + "/" + h for h in self.__class__.handles]
        self.contacts = [name + "/" + h for h in self.__class__.contacts]

class Box(object):
    handles = ["handle1", "handle2", "handle3", "handle4"]
    contacts = ["front_surface", "rear_surface"]
    rootJointType = "freeflyer"
    urdfFilename = "package://gerard_bauzil/urdf/plank_of_wood2.urdf"
    srdfFilename = "package://gerard_bauzil/srdf/plank_of_wood2.srdf"
    def __init__(self, name, vf):
        self.name = name
        vf.loadObjectModel(self.__class__, name)
        self.handles = [name + "/" + h for h in self.__class__.handles]
        self.contacts = [name + "/" + h for h in self.__class__.contacts]

HumanoidRobot.leftAnkle = "talos/leg_left_6_joint"
HumanoidRobot.rightAnkle = "talos/leg_right_6_joint"
HumanoidRobot.srdfString = ""

## Reduce joint range for security
def shrinkJointRange (robot, ratio):
    for j in robot.jointNames:
        if j [:6] == "talos/" and j [:13] != "talos/gripper":
            bounds = robot.getJointBounds (j)
            if len (bounds) == 2:
                width = bounds [1] - bounds [0]
                mean = .5 * (bounds [1] + bounds [0])
                m = mean - .5 * ratio * width
                M = mean + .5 * ratio * width
                robot.setJointBounds (j, [m, M])

def getSrdfString():
    import os
    package = 'talos_data'
    rosPackagePath = os.getenv('ROS_PACKAGE_PATH')
    paths = rosPackagePath.split(':')
    for p in paths:
        dir = p + '/' + package
        if os.path.isdir(dir):
            filename = dir + '/srdf/pyrene.srdf'
            if os.path.isfile(filename):
                with open(filename) as f:
                    res = f.read()
                    return res
    raise IOError('Could not open file ' + 'package://' + package + \
                  '/srdf/pyrene.srdf')

def makeRobotProblemAndViewerFactory(clients):
    try:
        import rospy
        HumanoidRobot.urdfString = rospy.get_param('robot_description')
        HumanoidRobot.srdfString = getSrdfString()
        print("reading URDF from ROS param")
    except:
        print("reading generic URDF")
        HumanoidRobot.urdfFilename = "package://talos_data/urdf/pyrene.urdf"
        HumanoidRobot.srdfFilename = "package://talos_data/srdf/pyrene.srdf"

    objects = list()
    robot = HumanoidRobot("talos", "talos", rootJointType="freeflyer", client=clients)
    robot.insertRobotSRDFModel("talos", "package://agimus_demos/srdf/contact_surface_on_the_arms.srdf")
    robot.leftAnkle = "talos/leg_left_6_joint"
    robot.rightAnkle = "talos/leg_right_6_joint"
    robot.camera_frame = 'talos/rgbd_rgb_optical_frame'
    robot.camera_frame = 'talos/head_d435_camera_color_optical_frame'
    robot.setJointBounds("talos/root_joint", [-2, 2, -2, 2, 0, 2])
    shrinkJointRange (robot, 0.95)

    ps = ProblemSolver(robot)
    ps.selectPathValidation('Progressive', 1e-3)
    ps.setErrorThreshold(1e-3)
    ps.setMaxIterProjection(40)
    ps.setDefaultLineSearchType('FixedSequence')

    vf = ViewerFactory(ps)

    table = Table(name="table", vf=vf)
    box   = Box  (name="box",   vf=vf)
    objects.append(box)

    robot.setJointBounds("table/root_joint", [-2, 2, -2, 2, -2, 2])
    robot.setJointBounds("box/root_joint"  , [-2, 2, -2, 2, -2, 2])

    return robot, ps, vf, table, objects

def makeGraph(ps, table, box, initConf):
    robot = ps.robot
    graph = ConstraintGraph(robot, "graph")
    ps.createQPStabilityConstraint('contact_on_left_arm/force', 'talos/root_joint',
        ['talos/left_arm','talos/left_sole','talos/right_sole'])
    ps.createQPStabilityConstraint('stand_on_feet/force', 'talos/root_joint',
        ['talos/left_sole','talos/right_sole'])
    createObjectLockedJoint(ps, table, initConf)
    leftPlace, leftPrePlace = ps.createPlacementConstraints\
        ('contact_on_left_arm/pose', ['talos/left_arm'], ['table/top'],
         width=.1)
    boxPlace, boxPrePlace = ps.createPlacementConstraints\
        ('place_box', ['box/front_surface'], ['table/top'], width=.15)
    graph.createPreGrasp('pregrasp_box', 'talos/right_gripper', 'box/handle3')

    leftGripperLock, rightGripperLock = createGripperLockedJoints(ps, initConf)
    com_constraint, footPlacement, footPlacementComplement = \
        createQuasiStaticEquilibriumConstraint (ps, initConf)
    lookLeftHand, lookRightHand = createGazeConstraints(ps)
    leftWristLock = createLeftWristLockJoint(ps, -0.6)
    # Constraints that are applied to all states
    commonStateConstraints = Constraints(numConstraints = footPlacement +\
                                         leftGripperLock + rightGripperLock +\
                                         [boxPlace,])
    # Constraints that are applied to all transitions
    commonTransitionConstraints = Constraints(numConstraints = \
        footPlacementComplement + ['table/root_joint', 'place_box/complement'])
    # Create states
    graph.createNode(['pregrasp_box', 'lean_on_left_arm', 'free',
                      'starting_state'])
    # Create transitions
    graph.createEdge('free', 'lean_on_left_arm', 'go_to_left_contact', 1,
                     isInNode='free')
    graph.createEdge('lean_on_left_arm', 'free', 'release_left_contact', 1,
                     isInNode='free')
    graph.createEdge('lean_on_left_arm', 'pregrasp_box', 'get_box', 1,
                     isInNode='lean_on_left_arm')
    graph.createEdge('pregrasp_box', 'lean_on_left_arm', 'get_away_from_box', 1,
                     isInNode='lean_on_left_arm')
    graph.createEdge("starting_state", "free", "starting_motion", isInNode="starting_state")
    graph.createEdge("free", "starting_state", "go_to_starting_state",
                     isInNode="starting_state",
                     weight=0,
                     )
    graph.createEdge('free', 'free', 'Loop | f', 1, isInNode='free')
    graph.createEdge('lean_on_left_arm', 'lean_on_left_arm',
                     'Loop | contact_on_left_arm', 1,
                     isInNode='lean_on_left_arm')
    # Set constraints in states and transitions
    graph.addConstraints(node='free', constraints = commonStateConstraints)
    graph.addConstraints(node='lean_on_left_arm',
        constraints = commonStateConstraints + Constraints(numConstraints = \
        [leftPlace, leftWristLock, lookLeftHand]))
    # Add force constraint at lower priority level
    # after initialization since the solvers need to be created
    cgraph = ps.client.basic.problem.getProblem().getConstraintGraph()
    nc = ps.client.basic.problem.getConstraint('stand_on_feet/force')
    state = cgraph.get(graph.nodes['free'])
    state.addNumericalCost(nc)
    state.setSolveLevelByLevel(True)

    nc = ps.client.basic.problem.getConstraint('contact_on_left_arm/force')
    state = cgraph.get(graph.nodes['lean_on_left_arm'])
    state.addNumericalCost(nc)
    state.setSolveLevelByLevel(True)
    for eid in graph.edges.values():
        edge = cgraph.get(eid)
        edge.setSolveLevelByLevel(True)

    graph.addConstraints(
        node='pregrasp_box',
        constraints = commonStateConstraints + \
        Constraints(numConstraints = [leftPlace, 'contact_on_left_arm/force',
            leftWristLock, lookLeftHand,'pregrasp_box']))

    graph.addConstraints(edge='go_to_left_contact', constraints =\
                        commonTransitionConstraints)
    graph.addConstraints(edge='Loop | f', constraints =\
                         commonTransitionConstraints)
    graph.addConstraints(edge='release_left_contact', constraints =\
                        commonTransitionConstraints)
    graph.addConstraints(
        edge="starting_motion",
        constraints=Constraints(numConstraints=['table/root_joint',]),)
    graph.addConstraints(
        edge="go_to_starting_state",
        constraints=Constraints(numConstraints=['table/root_joint',]))
    graph.addConstraints(
        edge='get_box', constraints = Constraints(
            numConstraints=['contact_on_left_arm/pose/complement',]) +
            commonTransitionConstraints)
    graph.addConstraints(
        edge='get_away_from_box', constraints = Constraints(
            numConstraints=['contact_on_left_arm/pose/complement',]) +
            commonTransitionConstraints)
    graph.addConstraints(
        edge='Loop | contact_on_left_arm', constraints = Constraints(
            numConstraints=['contact_on_left_arm/pose/complement',]) +
            commonTransitionConstraints)
    graph.initialize ()

    return graph

def shootConfig(robot, q, i):
    """
    Shoot a random config if i > 0, return input configuration otherwise
    """
    return q if i == 0 else robot.shootRandomConfig()


def createQuasiStaticEquilibriumConstraint (ps, q) :
    robot = ps.robot
    ps.addPartialCom("talos", ["talos/root_joint"])
    # Static stability constraint
    robot.createStaticStabilityConstraint(
        "balance/", "talos", robot.leftAnkle, robot.rightAnkle, q,
        (True, True, False)
    )
    com_constraint = ["balance/relative-com",]
    footPlacement = ["balance/pose-left-foot", "balance/pose-right-foot"]
    footPlacementComplement = []
    return com_constraint, footPlacement, footPlacementComplement


# Gaze constraints
def createGazeConstraints (ps):
    ps.createPositionConstraint(
        "look_left_hand",
        ps.robot.camera_frame,
        "talos/arm_left_7_joint",
        (0, 0, 0),
        (0, 0, -0.18),
        (True, True, False),
    )
    ps.createPositionConstraint(
        "look_right_hand",
        ps.robot.camera_frame,
        "talos/arm_right_7_joint",
        (0, 0, 0),
        (0, 0, -0.18),
        (True, True, False),
    )
    return ["look_left_hand", "look_right_hand"]

# Constraint of constant yaw of the waist
def createWaistYawConstraint (ps):
    ps.createOrientationConstraint(
        "waist_yaw", "", "talos/root_joint", (0, 0, 0, 1), [True, True, True]
    )
    ps.setConstantRightHandSide("waist_yaw", False)
    return ["waist_yaw"]

# Create locked joint for grippers
def createGripperLockedJoints (ps, q):
    leftGripperLock = list()
    rightGripperLock = list()
    for n in ps.robot.jointNames:
        s = ps.robot.getJointConfigSize(n)
        r = ps.robot.rankInConfiguration[n]
        if n.startswith("talos/gripper_right"):
            ps.createLockedJoint(n, n, q[r : r + s])
            ps.setConstantRightHandSide(n, True)
            rightGripperLock.append(n)
        elif n.startswith("talos/gripper_left"):
            ps.createLockedJoint(n, n, q[r : r + s])
            ps.setConstantRightHandSide(n, True)
            leftGripperLock.append(n)

    return leftGripperLock, rightGripperLock

# Create table and box locked joints
# only for easily generating pictures
def createTableAndBoxLockedJoints(ps, q):
    constraints = list()
    for n in ["table/root_joint", "box/root_joint"]:
        s = ps.robot.getJointConfigSize(n)
        r = ps.robot.rankInConfiguration[n]
        ps.createLockedJoint(n, n, q[r : r + s])
        ps.setConstantRightHandSide(n, True)
        constraints.append(n)
    return constraints

# Create locked joint for left wrist
def createLeftWristLockJoint(ps, value):
    n = 'talos/arm_left_7_joint'
    r = ps.robot.rankInConfiguration[n]
    ps.createLockedJoint(n, n, [value])
    ps.setConstantRightHandSide(n, True)
    return n

# Create locked joint for grippers and table
def createObjectLockedJoint (ps, obj, q):
    name = obj.name + "/root_joint"
    s = ps.robot.getJointConfigSize(name)
    r = ps.robot.rankInConfiguration[name]
    obj_lock = [name]
    ps.createLockedJoint(name, name, q[r : r + s])
    ps.setConstantRightHandSide(name, False)
    return obj_lock

# Set Gaussian shooter around input configuration with input variance
def setGaussianShooter (ps, table, objects, q_mean, sigma):
    robot = ps.robot
    # Set Gaussian configuration shooter.
    robot.setCurrentConfig(q_mean)
    # Set variance to 0.1 for all degrees of freedom
    u = robot.getNumberDof() * [sigma]
    # Set variance to 0.05 for robot free floating base
    rank = robot.rankInVelocity[robot.displayName + "/root_joint"]
    u[rank : rank + 6] = 6 * [0.0]
    # Set variance to 0.05 for box
    rank = robot.rankInVelocity[objects[0].name + "/root_joint"]
    u[rank : rank + 6] = 6 * [0.0]
    # Set variance to 0.05 for table
    rank = robot.rankInVelocity[table.name + "/root_joint"]
    u[rank : rank + 6] = 6 * [0.0]
    robot.setCurrentVelocity(u)
    ps.setParameter("ConfigurationShooter/Gaussian/useRobotVelocity", True)
    ps.client.basic.problem.selectConfigurationShooter("Gaussian")
    q_mean[robot.rankInConfiguration["box/root_joint"] + 1] += 0.1

def addCostToComponent(graph, costs, state=None, edge=None):
    if (state is None and edge is None) or (state is not None and edge is not None):
        raise ValueError ("Either state or edge arguments must be provided.")
    if state is not None:
        id = graph.states[state]
    else:
        id = graph.edges[edge]
    _problem = graph.clientBasic.problem.getProblem()
    _graph = _problem.getConstraintGraph()
    _comp = _graph.get(id)
    assert _comp.name() in [ state, edge ]
    _configConstraint = _comp.configConstraint()
    _cp = _configConstraint.getConfigProjector()
    _cp.setLastIsOptional(True)
    for cost in costs:
        _cost = graph.clientBasic.problem.getConstraint(cost)                                                                                                                                                 
        _cp.add(_cost, 1) 

# Shoot a random configuration for the right or left arm
# Keep all other degrees of freedom as in input configuration
def shootRandomArmConfig(robot, whichArm, q):
    if not whichArm in ['left', 'right']:
        raise RuntimeError('choose right or left arm.')
    res = robot.shootRandomConfig()
    prefix = 'talos/arm_' + whichArm
    for j in robot.jointNames:
        if j[:len(prefix)] != prefix:
            r = robot.rankInConfiguration[j]
            l = robot.getJointConfigSize(j)
            res[r:r+l] = q[r:r+l]
    return res

