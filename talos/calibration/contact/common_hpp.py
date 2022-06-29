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
from agimus_demos.talos.tools_hpp import wd, shrinkJointRange, getSrdfString

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
    camera_frame = 'talos/head_d435_camera_color_optical_frame'
    if not camera_frame in robot.getAllJointNames():
        print("Warning: the robot loaded does not have any 'talos/head_d435_camera_color_optical_frame'.")
        camera_frame = 'talos/head_t265_camera_link'
        if not camera_frame in robot.getAllJointNames():
            camera_frame = 'talos/rgbd_rgb_optical_frame'
        print("Assuming camera_frame is {}".format(camera_frame))

    robot.camera_frame = camera_frame
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
