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
from hpp.corbaserver.manipulation import ConstraintGraph, ProblemSolver, Rule
from hpp.corbaserver.manipulation.robot import HumanoidRobot
from hpp.gepetto.manipulation import ViewerFactory
from hpp.corbaserver.manipulation.constraint_graph_factory import ConstraintGraphFactory
from hpp.corbaserver.manipulation import SecurityMargins

defaultContext = "corbaserver"

class HPPObj(object):
    def __init__(self, name, vf):
        self.name = name
        vf.loadObjectModel(self.__class__, name)
        self.handles = [name + "/" + h for h in self.__class__.handles]
        self.contacts = [name + "/" + h for h in self.__class__.contacts]

    handles = []
    contacts = []


class Box(HPPObj):
    def __init__(self, name, vf):
        super(Box, self).__init__(name, vf)

    rootJointType = "freeflyer"
    packageName = "gerard_bauzil"
    urdfName = "plank_of_wood2"
    urdfSuffix = ""
    srdfSuffix = ""
    handles = ["handle1", "handle2", "handle3", "handle4"]
    contacts = ["front_surface", "rear_surface"]


class Table(HPPObj):
    def __init__(self, name, vf):
        super(Table, self).__init__(name, vf)

    rootJointType = "freeflyer"
    packageName = "gerard_bauzil"
    urdfName = "table_140_70_73"
    urdfSuffix = ""
    srdfSuffix = ""
    pose = "pose"
    handles = []
    contacts = ["top"]

class RollingTable(HPPObj):
    def __init__(self, name, vf):
        super(RollingTable, self).__init__(name, vf)

    rootJointType = "freeflyer"
    packageName = "gerard_bauzil"
    urdfName = "rolling_table"
    urdfSuffix = ""
    srdfSuffix = ""
    pose = "pose"
    handles = []
    contacts = ["top"]

rosInitialized = False
def ros_init():
    global rosInitialized
    if not rosInitialized:
        import rospy
        rospy.init_node("hpp_script", anonymous=True)
        rosInitialized = True

HumanoidRobot.urdfFilename = "package://talos_data/urdf/pyrene.urdf"
HumanoidRobot.srdfFilename = "package://talos_data/srdf/pyrene.srdf"

init_conf = json.load(open('../common/half_sitting.json', 'r'))
#init_conf[0:7] = [0.6, -0.65, 1.0192720229567027, 0, 0, sqrt(2) / 2, sqrt(2) / 2]  # root_joint
init_conf[0:7] = [0.1, -0.65, 1.0192720229567027, 0, 0, sqrt(2) / 2, sqrt(2) / 2]  # root_joint
init_conf += [-0.04, 0, 1.095 + 0.071, 0, 0, 1, 0, # box
               0, 0, 0, 0, 0, 0, 1] # table

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


def makeRobotProblemAndViewerFactory(clients, rolling_table=True, rosParam=None):
    if rosParam is not None:
        import rospy, os, hpp
        HumanoidRobot.urdfString = rospy.get_param(rosParam)
        srdfFilename = hpp.retrieveRosResource(HumanoidRobot.srdfFilename)
        with open(srdfFilename, 'r') as f:
            HumanoidRobot.srdfString = f.read()

    objects = list()
    robot = HumanoidRobot("talos", "talos", rootJointType="freeflyer", client=clients)
    robot.leftAnkle = "talos/leg_left_6_joint"
    robot.rightAnkle = "talos/leg_right_6_joint"
    robot.setJointBounds("talos/root_joint", [-2, 2, -2, 2, 0, 2])
    shrinkJointRange (robot, 0.95)

    ps = ProblemSolver(robot)
    ps.setErrorThreshold(1e-3)
    ps.setMaxIterProjection(40)
    ps.addPathOptimizer("EnforceTransitionSemantic")
    ps.addPathOptimizer("SimpleTimeParameterization")
    ps.selectPathValidation('Graph-Progressive', 0.01)

    vf = ViewerFactory(ps)

    objects.append(Box(name="box", vf=vf))
    robot.setJointBounds("box/root_joint", [-2, 2, -2, 2, 0, 2])

    # Loaded as an object to get the visual tags at the right position.
    # vf.loadEnvironmentModel (Table, 'table')
    if rolling_table:
        table = RollingTable(name="table", vf=vf)
    else:
        table = Table(name="table", vf=vf)
    robot.setJointBounds("table/root_joint", [-2, 2, -2, 2, -2, 2])

    return robot, ps, vf, table, objects

def makeRules (robot, grippers):
    handles = ['box/handle1', 'box/handle2', 'box/handle3', 'box/handle4']
    res = list ()
    res.append (Rule (grippers = grippers,
                      handles = ['^$', '^$',], link = True))
    for h in handles:
        res.append (Rule (grippers = grippers,
                          handles = [h, '^$',], link = True))
        res.append (Rule (grippers = grippers,
                          handles = ['^$', h,], link = True))
    res.append (Rule (grippers = grippers,
                      handles = [handles [0], handles [1]], link = True))
    res.append (Rule (grippers = grippers,
                      handles = [handles [1], handles [0]], link = True))
    res.append (Rule (grippers = grippers,
                      handles = [handles [0], handles [3]], link = True))
    res.append (Rule (grippers = grippers,
                      handles = [handles [3], handles [0]], link = True))
    res.append (Rule (grippers = grippers,
                      handles = [handles [2], handles [1]], link = True))
    res.append (Rule (grippers = grippers,
                      handles = [handles [1], handles [2]], link = True))
    res.append (Rule (grippers = grippers,
                      handles = [handles [2], handles [3]], link = True))
    res.append (Rule (grippers = grippers,
                      handles = [handles [3], handles [2]], link = True))
    return res

def makeGraph(ps, table, objects):
    robot = ps.robot
    graph = ConstraintGraph(robot, "graph")
    factory = ConstraintGraphFactory(graph)
    grippers = ["talos/left_gripper", "talos/right_gripper"]
    factory.setGrippers(grippers)
    factory.setObjects(
        [obj.name for obj in objects],
        [obj.handles for obj in objects],
        [obj.contacts for obj in objects],
    )
    factory.environmentContacts(table.contacts)
    factory.constraints.strict = True
    factory.setRules (makeRules (robot, grippers))
    factory.setPreplacementDistance("box", 0.1)
    factory.generate()
    sm = SecurityMargins(ps, factory, ["talos", "box", "table"])
    sm.setSecurityMarginBetween ("talos", "box", 0.02)
    sm.setSecurityMarginBetween ("box", "table", 0.03)
    sm.apply()
    return graph, factory


def shootConfig(robot, q, i):
    """
    Shoot a random config if i > 0, return input configuration otherwise
    """
    return q if i == 0 else robot.shootRandomConfig()


def createConnection(ps, graph, e, q, maxIter):
    """
    Try to build a path along a transition from a given configuration
    """
    for i in range(maxIter):
        q_rand = shootConfig(ps.robot, q, i)
        res, q1, err = graph.generateTargetConfig(e, q, q_rand)
        if not res:
            continue
        res, p, msg = ps.directPath(q, q1, True)
        if not res:
            continue
        ps.addConfigToRoadmap(q1)
        ps.addEdgeToRoadmap(q, q1, p, True)
        print("Success (i={0})".format(i))
        return p, q1
    print("Failed  (maxIter={0})".format(maxIter))
    return None, None


class Solver(object):
    """
    Solver that tries direct connections before calling RRT.
    """

    def __init__(
        self,
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
    ):
        self.ps = ps
        self.graph = graph
        self.e_l_l1 = e_l_l1
        self.e_l_r1 = e_l_r1
        self.e_l_l2 = e_l_l2
        self.e_l_r2 = e_l_r2
        self.e_l_l3 = e_l_l3
        self.e_l_r3 = e_l_r3
        self.e_l_l4 = e_l_l4
        self.e_l_r4 = e_l_r4
        self.e_l1 = e_l1
        self.e_r1 = e_r1
        self.e_l2 = e_l2
        self.e_r2 = e_r2
        self.e_l3 = e_l3
        self.e_r3 = e_r3
        self.e_l4 = e_l4
        self.e_r4 = e_r4
        self.e_l1_r2 = e_l1_r2
        self.e_l1_r4 = e_l1_r4
        self.e_r1_l2 = e_r1_l2
        self.e_r1_l4 = e_r1_l4
        self.e_l2_r1 = e_l2_r1
        self.e_l2_r3 = e_l2_r3
        self.e_r2_l1 = e_r2_l1
        self.e_r2_l3 = e_r2_l3
        self.e_r3_l4 = e_r3_l4
        self.e_r3_l2 = e_r3_l2
        self.e_l3_r4 = e_l3_r4
        self.e_l3_r2 = e_l3_r2
        self.e_l4_r1 = e_l4_r1
        self.e_l4_r3 = e_l4_r3
        self.e_r4_l1 = e_r4_l1
        self.e_r4_l3 = e_r4_l3
        self.q_init = q_init
        self.q_goal = q_goal

    def addWaypoints(self, config):
        """
        Add a waypoint to the free state with elbows (4th joint) at -1.7
        """
        # TODO: Rename for clarity. Only used for init and goal
        e = "Loop | f"
        robot = self.ps.robot
        rank1 = robot.rankInConfiguration["talos/arm_left_4_joint"]
        rank2 = robot.rankInConfiguration["talos/arm_right_4_joint"]
        q = config[::]
        # move elbows
        q[rank1] = -1.7
        q[rank2] = -1.7
        # Project q on state 'free'
        res, wp, err = self.graph.generateTargetConfig(e, config, q)
        if res:
            # test collision for wp
            res, msg = robot.isConfigValid(wp)
            if res:
                # call steering method
                res, p, msg = self.ps.directPath(config, wp, True)
                if res:
                    # add node and edge
                    self.ps.addConfigToRoadmap(wp)
                    self.ps.addEdgeToRoadmap(config, wp, p, True)
                    # store wp
                    return wp
        return config

    def tryDirectPaths(self, possibleConnections):
        """
        Add direct paths between pairs of configurations if possible
        """
        for q1, q2 in possibleConnections:
            if q1 and q2:
                res, p, msg = self.ps.directPath(q1, q2, True)
                if res:
                    print("direct connection succeeded")
                    self.ps.addEdgeToRoadmap(q1, q2, p, True)
                else:
                    print("failed direct connection: " + msg)
                    print("q1= " + str(q1))
                    print("q2= " + str(q2))

    def solve(self):
        assert (
            np.linalg.norm(np.array(self.q_init[-7:]) - np.array(self.q_goal[-7:])) # Assert that the table is not to be moved
            < 1e-7
        )
        start = datetime.now()
        q_l1 = q_r1 = q_l2 = q_r2 = q_l3 = q_r3 = q_l4 = q_r4 = None
        q_l1_r2 = None
        q_r2_l1 = None
        q_l1_r4 = None
        q_r4_l1 = None
        q_r1_l2 = None
        q_l2_r1 = None
        q_r1_l4 = None
        q_l4_r1 = None
        q_l2_r3 = None
        q_r3_l2 = None
        q_r2_l3 = None
        q_l3_r2 = None
        q_l3_r4 = None
        q_r4_l3 = None
        q_r3_l4 = None
        q_l4_r3 = None
        self.ps.addConfigToRoadmap(self.q_init)
        self.ps.addConfigToRoadmap(self.q_goal)

        print("Generating init waypoint.")
        self.wp_init = self.addWaypoints(self.q_init)
        print("Generating goal waypoint.")
        self.wp_goal = self.addWaypoints(self.q_goal)

        ## Connections from init to grasp
        print("Edge e_l1   : ")
        p, q_l1 = createConnection(self.ps, self.graph, self.e_l1, self.wp_init, 20)
        print("Edge e_r1   : ")
        p, q_r1 = createConnection(self.ps, self.graph, self.e_r1, self.wp_init, 20)
        print("Edge e_l2   : ")
        p, q_l2 = createConnection(self.ps, self.graph, self.e_l2, self.wp_init, 20)
        print("Edge e_r2   : ")
        p, q_r2 = createConnection(self.ps, self.graph, self.e_r2, self.wp_init, 20)
        print("Edge e_l3   : ")
        p, q_l3 = createConnection(self.ps, self.graph, self.e_l3, self.wp_init, 20)
        print("Edge e_r3   : ")
        p, q_r3 = createConnection(self.ps, self.graph, self.e_r3, self.wp_init, 20)
        print("Edge e_l4   : ")
        p, q_l4 = createConnection(self.ps, self.graph, self.e_l4, self.wp_init, 20)
        print("Edge e_r4   : ")
        p, q_r4 = createConnection(self.ps, self.graph, self.e_r4, self.wp_init, 20)
        ## Connections from goal to grasp
        if q_l1 is None:
            print("Edge e_l1   : ")
            p, q_l1 = createConnection(self.ps, self.graph, self.e_l1, self.wp_goal, 20)
        if q_r1 is None:
            print("Edge e_r1   : ")
            p, q_r1 = createConnection(self.ps, self.graph, self.e_r1, self.wp_goal, 20)
        if q_l2 is None:
            print("Edge e_l2   : ")
            p, q_l2 = createConnection(self.ps, self.graph, self.e_l2, self.wp_goal, 20)
        if q_r2 is None:
            print("Edge e_r2   : ")
            p, q_r2 = createConnection(self.ps, self.graph, self.e_r2, self.wp_goal, 20)
        if q_l3 is None:
            print("Edge e_l3   : ")
            p, q_l3 = createConnection(self.ps, self.graph, self.e_l3, self.wp_goal, 20)
        if q_r3 is None:
            print("Edge e_r3   : ")
            p, q_r3 = createConnection(self.ps, self.graph, self.e_r3, self.wp_goal, 20)
        if q_l4 is None:
            print("Edge e_l4   : ")
            p, q_l4 = createConnection(self.ps, self.graph, self.e_l4, self.wp_goal, 20)
        if q_r4 is None:
            print("Edge e_r4   : ")
            p, q_r4 = createConnection(self.ps, self.graph, self.e_r4, self.wp_goal, 20)
        ## Connections from one grasp to two grasps
        if q_l1:
            print("Edge e_l1_r2: ")
            p, q_l1_r2 = createConnection(self.ps, self.graph, self.e_l1_r2, q_l1, 20)
            print("Edge e_l1_r4: ")
            p, q_l1_r4 = createConnection(self.ps, self.graph, self.e_l1_r4, q_l1, 20)
        if q_r1:
            print("Edge e_r1_l2: ")
            p, q_r1_l2 = createConnection(self.ps, self.graph, self.e_r1_l2, q_r1, 20)
            print("Edge e_r1_l4: ")
            p, q_r1_l4 = createConnection(self.ps, self.graph, self.e_r1_l4, q_r1, 20)
        if q_l2:
            print("Edge e_l2_r1: ")
            p, q_l2_r1 = createConnection(self.ps, self.graph, self.e_l2_r1, q_l2, 20)
            print("Edge e_l2_r3: ")
            p, q_l2_r3 = createConnection(self.ps, self.graph, self.e_l2_r3, q_l2, 20)
        if q_r2:
            print("Edge e_r2_l1: ")
            p, q_r2_l1 = createConnection(self.ps, self.graph, self.e_r2_l1, q_r2, 20)
            print("Edge e_r2_l3: ")
            p, q_r2_l3 = createConnection(self.ps, self.graph, self.e_r2_l3, q_r2, 20)
        if q_l3:
            print("Edge e_l3_r4: ")
            p, q_l3_r4 = createConnection(self.ps, self.graph, self.e_l3_r4, q_l3, 20)
            print("Edge e_l3_r2: ")
            p, q_l3_r2 = createConnection(self.ps, self.graph, self.e_l3_r2, q_l3, 20)
        if q_r3:
            print("Edge e_r3_l4: ")
            p, q_r3_l4 = createConnection(self.ps, self.graph, self.e_r3_l4, q_r3, 20)
            print("Edge e_r3_l2: ")
            p, q_r3_l2 = createConnection(self.ps, self.graph, self.e_r3_l2, q_r3, 20)
        if q_l4:
            print("Edge e_l4_r1: ")
            p, q_l4_r1 = createConnection(self.ps, self.graph, self.e_l4_r1, q_l4, 20)
            print("Edge e_l4_r3: ")
            p, q_l4_r3 = createConnection(self.ps, self.graph, self.e_l4_r3, q_l4, 20)
        if q_r4:
            print("Edge e_r4_l1: ")
            p, q_r4_l1 = createConnection(self.ps, self.graph, self.e_r4_l1, q_r4, 20)
            print("Edge e_r4_l3: ")
            p, q_r4_l3 = createConnection(self.ps, self.graph, self.e_r4_l3, q_r4, 20)

        possibleConnections = [
            (q_l1_r2, q_r2_l1),
            (q_l1_r4, q_r4_l1),
            (q_r1_l2, q_l2_r1),
            (q_r1_l4, q_l4_r1),
            (q_l2_r3, q_r3_l2),
            (q_r2_l3, q_l3_r2),
            (q_l3_r4, q_r4_l3),
            (q_r3_l4, q_l4_r3),
        ]
        self.tryDirectPaths(possibleConnections)

        self.ps.setInitialConfig(self.q_init)
        self.ps.resetGoalConfigs()
        self.ps.addGoalConfig(self.q_goal)
        print("Running Manipulation RRT")
        self.ps.solve()
        end = datetime.now()
        print("Resolution time : {0}".format(end - start))

    def makeBoxVisibleFrom(self, q_estimated, initObjectPose, initTablePose):
        # TODO: Doc + rename vars for clarity
        q = q_estimated[:]
        rank_box = self.ps.robot.rankInConfiguration["box/root_joint"]
        rank_table = self.ps.robot.rankInConfiguration["table/root_joint"]
        if initObjectPose:
            q[rank_box : rank_box + 7] = self.q_init[rank_box : rank_box + 7]
        if initTablePose:
            q[rank_table : rank_table + 7] = self.q_init[rank_table : rank_table + 7]

        res, q_proj, err = self.graph.generateTargetConfig("loop_ss", q, q)
        assert res, "Failed: generateTargetConfig loop_ss"
        res, qres, err = self.graph.generateTargetConfig(
            "starting_motion", q_proj, q_proj
        )
        assert res, "Failed: generateTargetConfig starting_motion"
        res, pid, msg = self.ps.directPath(q_proj, qres, True)
        self.ps.addConfigToRoadmap(q_proj)
        self.ps.addConfigToRoadmap(qres)
        self.ps.addEdgeToRoadmap(q_proj, qres, pid, True)
        if res:
            self.ps.optimizePath(pid)
            print("Initial path", pid + 1)
            return qres, pid + 1
        else:
            print("Failed: directPath", msg)
            return qres, None

    # \param qEstimated box should be visible.
    def generateGoalFrom(self, qEstimated, qDesiredRobot):
        qgoal = qEstimated[:]
        rankO = self.ps.robot.rankInConfiguration["box/root_joint"]
        rankT = self.ps.robot.rankInConfiguration["table/root_joint"]
        # Rotate the box.

        qT = Quaternion(qEstimated[rankO + 3 : rankO + 7])
        qgoal[rankO + 3 : rankO + 7] = (qT * Quaternion([0, 0, 1, 0])).toTuple()
        res, qgoal, err = self.graph.generateTargetConfig(
            "starting_motion", qgoal, qgoal
        )
        success = "free" == self.graph.getNode(qgoal)
        if not res or not success:
            print("Could not generate goal")
        qgoalInStartingState = (
            qDesiredRobot[: min(rankO, rankT)] + qgoal[min(rankO, rankT) :]
        )
        # TODO if a transition from free to starting_state is added
        # change the order of the arguments.
        # res, pid, msg = ps.directPath (qgoal, qgoalInStartingState, True)
        # self.tryDirectPaths (((qgoalInStartingState, qgoal),))
        return qgoal, qgoalInStartingState

    # \param qEstimated box should be visible.
    def generateLeftHandGraspFrom(self, qEstimated):
        qgoals = []

        ## Connections from init to grasp
        print("Edge e_l1   : ")
        p, q_l1 = createConnection(self.ps, self.graph, self.e_l1, self.wp_init, 20)
        print("Edge e_l2   : ")
        p, q_l2 = createConnection(self.ps, self.graph, self.e_l2, self.wp_init, 20)
        print("Edge e_l3   : ")
        p, q_l3 = createConnection(self.ps, self.graph, self.e_l3, self.wp_init, 20)
        print("Edge e_l4   : ")
        p, q_l4 = createConnection(self.ps, self.graph, self.e_l4, self.wp_init, 20)
        if q_l1: qgoals.append (q_l1)
        if q_l2: qgoals.append (q_l2)
        if q_l3: qgoals.append (q_l3)
        if q_l4: qgoals.append (q_l4)

        return qgoals

    def acquireEstimation(self, topic="/agimus/estimation/semantic"):
        boxSizeZ = 0.203
        tableRealHeight = 0.74
        boxExpectedZ = tableRealHeight + boxSizeZ / 2

        ros_init()
        import rospy
        from dynamic_graph_bridge_msgs.msg import Vector
        msg = rospy.wait_for_message(topic, Vector, timeout=2)
        qestimated = list(msg.data)
        # Fix estimation
        self.ps.robot.setCurrentConfig(qestimated)
        p_cam = np.array(
            self.ps.robot.getJointPosition("talos/rgbd_rgb_optical_joint")[0:3]
        )
        p_obj = np.array(self.ps.robot.getJointPosition("box/root_joint")[0:3])
        p_cam_obj = p_cam - p_obj

        ric = self.ps.robot.rankInConfiguration
        qestimated[ric["talos/gripper_right_joint"]] = 0
        qestimated[ric["talos/gripper_left_joint"]] = 0
        curBoxZ = qestimated[ric["box/root_joint"] + 2]
        z_corr = boxExpectedZ - curBoxZ + 0.01

        x_corr = z_corr / p_cam_obj[2] * p_cam_obj[0]
        y_corr = z_corr / p_cam_obj[2] * p_cam_obj[1] - 0.026 # TODO: Check this magic value!
        print("Correction on X (NOT applied) axis:", x_corr)
        print("Correction on Y (NOT applied) axis:", y_corr)
        print("Correction on Z (NOT applied) axis:", z_corr)
        # qestimated[ric["box/root_joint"]  +0] +=x_corr
        # qestimated[ric["table/root_joint"]+0] +=x_corr
        #qestimated[ric["box/root_joint"] + 1] += y_corr
        #qestimated[ric["table/root_joint"] + 1] += y_corr
        #qestimated[ric["box/root_joint"] + 2] += z_corr
        #qestimated[ric["table/root_joint"] + 2] += z_corr
        # TODO: We can also correct with the estimation of the table height and ORIENTATION against the real values

        return qestimated

    def solveFromEstimatedConfiguration(self, half_sitting, q_estimated=None):
        if q_estimated is None:
            self.q_estimated = self.acquireEstimation()
        else:
            self.q_estimated = q_estimated
        self.q_init, initial_path_id = self.makeBoxVisibleFrom(
            self.q_estimated, False, False
        )
        self.q_goal, self.q_goal_in_starting_state = self.generateGoalFrom(
            self.q_estimated, half_sitting
        )
        self.solve()
        print("Initial path: ", initial_path_id)
        print("Path to achieve the box goal position: ", self.ps.numberPaths() - 1)
        res, pid, msg = self.ps.directPath(
            self.q_goal, self.q_goal_in_starting_state, True
        )
        if res:
            self.ps.optimizePath(pid)
            print("Path to go back to half_sitting:", self.ps.numberPaths() - 1)
        else:
            print("Cannot go back to half_sitting:", msg)

    def graspBoxWithLeftHand(self, q_estimated=None):
        if q_estimated is None:
            self.q_estimated = self.acquireEstimation()
        else:
            self.q_estimated = q_estimated
        # Look at the box
        self.q_init, initial_path_id = self.makeBoxVisibleFrom(
            self.q_estimated, False, False
        )
        # Move hands up.
        self.ps.addConfigToRoadmap(self.q_init)
        print("Generating init waypoint.")
        self.wp_init = self.addWaypoints(self.q_init)

        self.q_goals = self.generateLeftHandGraspFrom(self.wp_init)

        if not self.q_goals:
            raise RuntimeError("Failed to generate goal configs")

        self.ps.setInitialConfig(self.q_init)
        self.ps.resetGoalConfigs()
        for q_goal in self.q_goals:
            # self.ps.addConfigToRoadmap(self.q_goal)
            self.ps.addGoalConfig(q_goal)

        duration = self.ps.solve()
        print("Resolution time : {0}h{1}m{2}s{3}us".format(*duration))
        print("Initial path: ", initial_path_id)
        print("Path to achieve the box goal position: ", self.ps.numberPaths() - 1)

    def goTo(self, half_sitting):
        # TODO: Clean this mess
        q_current = self.acquireEstimation()
        q_init = q_current[:]
        rankO = self.ps.robot.rankInConfiguration["box/root_joint"]
        rankT = self.ps.robot.rankInConfiguration["table/root_joint"]
        q_init[rankT : rankT + 7] = self.q_init[rankT : rankT + 7]
        q_init[rankO : rankO + 7] = self.q_init[rankO : rankO + 7]
        self.ps.setInitialConfig(q_init)
        self.ps.resetGoalConfigs()
        q_goal = half_sitting
        q_goal[rankT : rankT + 7] = self.q_init[rankT : rankT + 7]
        q_goal[rankO : rankO + 7] = self.q_init[rankO : rankO + 7]
        self.ps.addGoalConfig(q_goal)
        print(self.ps.solve())

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


# Gaze constraint
def createGazeConstraint (ps):
    ps.createPositionConstraint(
        "gaze",
        "talos/rgbd_optical_joint",
        "box/root_joint",
        (0, 0, 0),
        (0, 0, 0),
        (True, True, False),
    )
    return ["gaze"]

# Gaze constraints
def createGazeConstraints (ps):
    ps.createPositionConstraint(
        "look_left_hand",
        "talos/rgbd_optical_joint",
        "talos/arm_left_7_joint",
        (0, 0, 0),
        (0, 0, -0.18),
        (True, True, False),
    )
    ps.createPositionConstraint(
        "look_right_hand",
        "talos/rgbd_optical_joint",
        "talos/arm_right_7_joint",
        (0, 0, 0),
        (0, 0, -0.18),
        (True, True, False),
    )
    return ["look_left_hand", "look_right_hand"]

# Gaze cost
def createGazeCost (ps):
    ps.createPositionConstraint(
        "gaze_cost",
        "talos/rgbd_optical_joint",
        "box/root_joint",
        (0, 0, 0.4),
        (0, 0, 0),
        (False, False, True),
    )
    return ["gaze_cost"]

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
            right_gripper_lock.append(n)
        elif n.startswith("talos/gripper_left"):
            ps.createLockedJoint(n, n, q[r : r + s])
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
    # Set variance to 0.0 for box
    rank = robot.rankInVelocity[objects[0].name + "/root_joint"]
    u[rank : rank + 6] = 6 * [0.0]
    # Set variance to 0.0 for table
    rank = robot.rankInVelocity[table.name + "/root_joint"]
    u[rank : rank + 6] = 6 * [0.0]
    # Set variance to 0.0 for head
    rank = robot.rankInVelocity[robot.displayName + '/head_1_joint']
    u[rank : rank + 2] = 2* [0.0]

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
