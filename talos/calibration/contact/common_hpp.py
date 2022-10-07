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
    contacts = ["top", "side", 'front']
    handles = ["contact_01", "contact_02", "contact_03",
                "contact_04", "contact_05", "contact_06",
                "contact_07", "contact_08", "contact_09",
                "contact_10", "contact_11", "contact_12",
                "contact_13", "contact_14", "contact_15",
                "contact_16", "contact_17", "contact_18",
                "contact_19", "contact_20", "contact_21",
                "contact_22", "contact_23", "contact_24",
                "contact_25", "contact_26", "contact_27",]
    # handles = ["contact_01"]
    rootJointType = "freeflyer"
    urdfFilename = "package://desk/urdf/desk.urdf"
    srdfFilename = "package://desk/srdf/desk_calibration_contact.srdf"

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
    ps.setErrorThreshold(1e-4)
    ps.setMaxIterProjection(40)

    vf = ViewerFactory(ps)

    table = Table(name="table", vf=vf)
    robot.setJointBounds("table/root_joint", [-2, 2, -2, 2, -2, 2])

    return robot, ps, vf, table, objects

def makeGraph(ps, table, graph, autoCollision = False):
    robot = ps.robot
    factory = ConstraintGraphFactory(graph)
    grippers = ["talos/left_gripper", "talos/right_gripper"]
    factory.setGrippers(grippers)
    factory.setObjects([table.name,], [table.handles,], [table.contacts,])
    talosBody = ['talos/root_joint', 'talos/leg_left_1_joint',
                 'talos/leg_left_2_joint', 'talos/leg_left_3_joint',
                 'talos/leg_left_4_joint', 'talos/leg_left_5_joint',
                 'talos/leg_left_6_joint', 'talos/leg_right_1_joint',
                 'talos/leg_right_2_joint', 'talos/leg_right_3_joint',
                 'talos/leg_right_4_joint', 'talos/leg_right_5_joint',
                 'talos/leg_right_6_joint', 'talos/torso_1_joint',
                 'talos/torso_2_joint', 'talos/head_1_joint',
                 'talos/head_2_joint',]
    talosLeftArm = ['talos/arm_left_6_joint', 'talos/arm_left_7_joint',
                    'talos/gripper_left_inner_double_joint',
                    'talos/gripper_left_fingertip_1_joint',
                    'talos/gripper_left_fingertip_2_joint',
                    'talos/gripper_left_inner_single_joint',
                    'talos/gripper_left_fingertip_3_joint',
                    'talos/gripper_left_joint',
                    'talos/gripper_left_motor_single_joint',]
    talosRightArm = ['talos/arm_right_6_joint', 'talos/arm_right_7_joint',
                     'talos/gripper_right_inner_double_joint',
                     'talos/gripper_right_fingertip_1_joint',
                     'talos/gripper_right_fingertip_2_joint',
                     'talos/gripper_right_inner_single_joint',
                     'talos/gripper_right_fingertip_3_joint',
                     'talos/gripper_right_joint',
                     'talos/gripper_right_motor_single_joint',]
    # forbid contact with 2 hands
    rules = [Rule(grippers=grippers, handles=(table.name + "/contact_*",
                                              table.name + "/contact_*"),
                  link=False),
             Rule(grippers=grippers, handles=(".*", ".*"), link=True)]
    factory.setRules(rules)
    factory.generate()
    sm = SecurityMargins(ps, factory, ["talos", "table"])
    sm.setSecurityMarginBetween("talos", "table", 0.04)
    sm.setSecurityMarginBetween("talos", "talos", 0)
    sm.defaultMargin = 0.01
    sm.apply()

    # Set bigger margins between some pairs of talos bodies
    cproblem = wd(ps.client.basic.problem.getProblem())
    cgraph = wd(cproblem.getConstraintGraph())
    if autoCollision:
        for j1 in talosBody:
            for j2 in talosLeftArm + talosRightArm:
                for e in list(graph.edges.values()):
                    cedge = wd(cgraph.get(e))
                    cedge.setSecurityMarginForPair(robot.jointNames.index(j1)+1,
                                                   robot.jointNames.index(j1)+1,
                                                   0.01)

    # deactivate collision checking between fingertips and table between
    # pregrasp and grasp
    for ig, g in enumerate(factory.grippers):
        joint, _ = robot.getGripperPositionInJoint(g)
        if joint[10:14] == 'left':
            side = 'left'
        elif joint[10:15] == 'right':
            side = 'right'
        else:
            raise RuntimeError('Failed to recognize wrist joint "{}"'.\
                               format(joint))
        fingers = ["talos/gripper_{}_joint",
                   "talos/gripper_{}_inner_double_joint",
                   "talos/gripper_{}_fingertip_1_joint",
                   "talos/gripper_{}_fingertip_2_joint",
                   "talos/gripper_{}_motor_single_joint",
                   "talos/gripper_{}_inner_single_joint",
                   "talos/gripper_{}_fingertip_3_joint",]
        for ih, h in enumerate(factory.handles):
            name = "{} > {} | f_12".format(g,h)
            cedge = wd(cgraph.get(graph.edges[name]))
            for finger in fingers:
                cedge.setSecurityMarginForPair(robot.jointNames.index\
                    (finger.format(side))+1, robot.jointNames.index\
                                               (table.name + '/root_joint')+1,
                                               float('-inf'))
            name = "Loop | {}-{}".format(ig,ih)
            cedge = wd(cgraph.get(graph.edges[name]))
            for finger in fingers:
                cedge.setSecurityMarginForPair(robot.jointNames.index\
                    (finger.format(side))+1, robot.jointNames.index\
                                               (table.name + '/root_joint')+1,
                                               float('-inf'))
            name = "{} < {} | {}-{}_21".format(g, h, ig,ih)
            cedge = wd(cgraph.get(graph.edges[name]))
            for finger in fingers:
                cedge.setSecurityMarginForPair(robot.jointNames.index\
                    (finger.format(side))+1, robot.jointNames.index\
                                               (table.name + '/root_joint')+1,
                                               float('-inf'))

def createQuasiStaticEquilibriumConstraint (ps, q) :
    robot = ps.robot
    ps.addPartialCom("talos", ["talos/root_joint"])
    # Static stability constraint
    robot.createStaticStabilityConstraint(
        "balance/", "talos", robot.leftAnkle, robot.rightAnkle, q,
        maskCom = [True, True, False]
    )
    com_constraint = ["balance/relative-com",]
    foot_placement = ["balance/pose-left-foot", "balance/pose-right-foot"]
    foot_placement_complement = []
    return com_constraint, foot_placement, foot_placement_complement
