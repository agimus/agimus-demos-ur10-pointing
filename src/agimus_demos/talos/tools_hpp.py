# Copyright 2022 CNRS - Airbus SAS
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

from hpp.corbaserver import wrap_delete, Client

defaultContext = "corbaserver"
client = Client()

def wd(o):
    return wrap_delete(o, client._tools)

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
def setGaussianShooter (ps, objects, q_mean, sigma, variance = dict()):
    robot = ps.robot
    # Set Gaussian configuration shooter.
    ps.setParameter('ConfigurationShooter/Gaussian/center',q_mean)
    # Set variance to all degrees of freedom
    u = robot.getNumberDof() * [sigma]
    # Set variance to  robot free floating base
    name = robot.displayName + "/root_joint"
    _sigma = variance.get(name, sigma)
    rank = robot.rankInVelocity[name]
    u[rank : rank + 6] = 6 * [_sigma]
    # Set variance to objects
    for o in objects:
        name = o.name + "/root_joint"
        _sigma = variance.get(name, sigma)
        rank = robot.rankInVelocity[name]
        u[rank : rank + 6] = 6 * [_sigma]
    robot.setCurrentVelocity(u)
    ps.setParameter('ConfigurationShooter/Gaussian/useRobotVelocity', True)
    ps.setParameter('ConfigurationShooter/Gaussian/center', q_mean)
    ps.client.basic.problem.selectConfigurationShooter("Gaussian")

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

