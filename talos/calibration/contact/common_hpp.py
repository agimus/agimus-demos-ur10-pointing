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
    SecurityMargins
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
    handles = ["contact_01", "contact_02", "contact_03", "contact_04"]
    handles = ["contact_01"]
    rootJointType = "freeflyer"
    urdfFilename = "package://gerard_bauzil/urdf/rolling_table.urdf"
    srdfFilename = "package://agimus_demos/srdf/rolling_table_calibration_contact.srdf"

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
    robot.leftAnkle = "talos/leg_left_6_joint"
    robot.rightAnkle = "talos/leg_right_6_joint"
    robot.camera_frame = 'talos/rgbd_rgb_optical_frame'
    robot.setJointBounds("talos/root_joint", [-2, 2, -2, 2, 0, 2])
    shrinkJointRange (robot, 0.95)

    ps = ProblemSolver(robot)
    ps.selectPathValidation('Progressive', 1e-3)
    ps.setErrorThreshold(1e-3)
    ps.setMaxIterProjection(40)

    vf = ViewerFactory(ps)

    table = Table(name="table", vf=vf)
    robot.setJointBounds("table/root_joint", [-2, 2, -2, 2, -2, 2])

    return robot, ps, vf, table, objects

def makeGraph(ps, table):
    robot = ps.robot
    graph = ConstraintGraph(robot, "graph")
    factory = ConstraintGraphFactory(graph)
    grippers = ["talos/left_gripper", "talos/right_gripper"]
    factory.setGrippers(grippers)
    factory.setObjects([table.name,], [table.handles,], [table.contacts,])
    # forbid contact with 2 hands
    rules = [Rule(grippers=grippers, handles=(table.name + "/contact_*",
                                              table.name + "/contact_*"),
                  link=False),
             Rule(grippers=grippers, handles=(".*", ".*"), link=True)]
    factory.setRules(rules)
    factory.generate()
    sm = SecurityMargins(ps, factory, ["talos", "table"])
    sm.setSecurityMarginBetween("talos", "table", 0.05)
    sm.setSecurityMarginBetween("talos", "talos", 0)
    sm.defaultMargin = 0.01
    sm.apply()
    # deactivate collision checking between fingertips and table between
    # pregrasp and grasp
    cproblem = wd(ps.client.basic.problem.getProblem())
    cgraph = wd(cproblem.getConstraintGraph())
    for ig, g in enumerate(factory.grippers):
        joint, _ = robot.getGripperPositionInJoint(g)
        if joint[10:14] == 'left':
            side = 'left'
        elif joint[10:15] == 'right':
            side = 'right'
        else:
            raise RuntimeError('Failed to recognize wrist joint "{}"'.\
                               format(joint))
        for ih, h in enumerate(factory.handles):
            name = "{} > {} | f_12".format(g,h)
            cedge = wd(cgraph.get(graph.edges[name]))
            for i in range(1,4):
                fingertip = "talos/gripper_{}_fingertip_{}_joint".format\
                            (side, i)
                cedge.setSecurityMarginForPair(robot.jointNames.index(fingertip)
                    +1, robot.jointNames.index(table.name + '/root_joint')+1,
                    float('-inf'))
            name = "Loop | {}-{}".format(ig,ih)
            cedge = wd(cgraph.get(graph.edges[name]))
            for i in range(1,4):
                fingertip = "talos/gripper_{}_fingertip_{}_joint".format\
                            (side, i)
                cedge.setSecurityMarginForPair(robot.jointNames.index(fingertip)
                    +1, robot.jointNames.index(table.name + '/root_joint')+1,
                    float('-inf'))
            name = "{} < {} | {}-{}_21".format(g, h, ig,ih)
            cedge = wd(cgraph.get(graph.edges[name]))
            for i in range(1,4):
                fingertip = "talos/gripper_{}_fingertip_{}_joint".format\
                            (side, i)
                cedge.setSecurityMarginForPair(robot.jointNames.index(fingertip)
                    +1, robot.jointNames.index(table.name + '/root_joint')+1,
                    float('-inf'))
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
        "balance/", "talos", robot.leftAnkle, robot.rightAnkle, q
    )
    com_constraint = ["balance/relative-com",]
    foot_placement = ["balance/pose-left-foot", "balance/pose-right-foot"]
    foot_placement_complement = []
    return com_constraint, foot_placement, foot_placement_complement


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

# Create locked joint for left arm
def createLeftArmLockedJoints (ps):
    left_arm_lock = list()
    for n in ps.robot.jointNames:
        if n.startswith("talos/arm_left"):
            ps.createLockedJoint(n, n, [0])
            ps.setConstantRightHandSide(n, False)
            left_arm_lock.append(n)
    return left_arm_lock

# Create locked joint for right arm
def createRightArmLockedJoints (ps):
    right_arm_lock = list()
    for n in ps.robot.jointNames:
        if n.startswith("talos/arm_right"):
            ps.createLockedJoint(n, n, [0])
            ps.setConstantRightHandSide(n, False)
            right_arm_lock.append(n)
    return right_arm_lock

# Create locked joint for grippers
def createGripperLockedJoints (ps, q):
    left_gripper_lock = list()
    right_gripper_lock = list()
    for n in ps.robot.jointNames:
        s = ps.robot.getJointConfigSize(n)
        r = ps.robot.rankInConfiguration[n]
        if n.startswith("talos/gripper_right"):
            ps.createLockedJoint(n, n, q[r : r + s])
            ps.setConstantRightHandSide(n, True)
            right_gripper_lock.append(n)
        elif n.startswith("talos/gripper_left"):
            ps.createLockedJoint(n, n, q[r : r + s])
            ps.setConstantRightHandSide(n, True)
            left_gripper_lock.append(n)

    return left_gripper_lock, right_gripper_lock

# Create locked joint for grippers and table
def createTableLockedJoint (ps, table, q):
    name = table.name + "/root_joint"
    s = ps.robot.getJointConfigSize(name)
    r = ps.robot.rankInConfiguration[name]
    table_lock = [name]
    ps.createLockedJoint(name, name, q[r : r + s])
    ps.setConstantRightHandSide(name, False)
    return table_lock

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

