# /usr/bin/env python

import math
import random

import rosbag
from hpp import Transform
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.corbaserver.manipulation.robot import CorbaClient, Robot
from hpp.gepetto.manipulation import ViewerFactory

# We have a set of observations:
# - oMi_j = oMi(q_j) <-> Measured transformation matrix from the Origin to the body i, number j. It's a function of the configuration q_j
# - oMk_j <-> Measured transformation matrix from the Origin to the object/tag k, number j
# And a set of unknowns:
# - C_i <-> Transformation matrix between the measured transformation matrix oMi and the real pose of the body i
# - iTk <-> Transformation matrix between the real pose of the body i and the object/tag (real or measured is irrelevant, because we can write iTk = iTk_real * k_realT_measured and we are searching for C_i)

# So we have oMk_j^-1 * oMi(q_j) * C_i * iTk = Identity
# Which, in SE(3), can be written C( j, C_i, iTk ) = log( oMk_j^-1 * oMi(q_j) * C_i * iTk ) = 0

# The problem is then to minimize the sum, over j and k,  of || C( j, C_i, iTk ) ||^2

client = CorbaClient()


class OtherRobot(object):
    def __init__(self, name, vf):
        self.name = name
        self.handles = [name + "/" + h for h in self.__class__.handles]
        self.contacts = [name + "/" + h for h in self.__class__.contacts]
        vf.loadObjectModel(self.__class__, name)

    rootJointType = "freeflyer"
    packageName = "agimus_demos"
    urdfName = "talos"
    urdfSuffix = "_calibration_camera"
    srdfSuffix = ""
    handles = []
    contacts = []


class Calibration(object):
    def __init__(self, client):
        self.client = client
        self.robot = None
        self.robots = list()
        self.robot_locks = list()
        self.robots_locks = list()
        self.robots_gaze = list()
        self.robots_identity_constraints = list()
        self.q = []

    def initRobot(self):
        # The first robot is loaded differently by hpp
        robot_id = len(self.robots) + int(self.robot is not None)
        robot_name = "talos_" + str(robot_id)

        if self.robot is None:
            Robot.packageName = "agimus_demos"
            Robot.urdfName = "talos"
            Robot.urdfSuffix = "_calibration_camera"
            Robot.srdfSuffix = ""

            self.robot = Robot(
                "talos", robot_name, rootJointType="freeflyer", client=self.client
            )

            self.ps = ProblemSolver(self.robot)
            self.ps.setErrorThreshold(0.00000001)
            self.ps.setMaxIterProjection(100000)

            self.vf = ViewerFactory(self.ps)
        else:
            self.robots.append(OtherRobot(name=robot_name, vf=self.vf))

        self.robot.setJointBounds(robot_name + "/root_joint", [-1, 1, -1, 1, 0.5, 1.5])

        rank, size = self.getRankAndConfigSize(robot_id)
        self.q[rank : rank + size] = self.robot.getCurrentConfig()[rank : rank + size]

        return robot_id

    def setLockJoints(self, robot_id, joints_name_value_tuple):
        self.ps.createLockedJoint(
            "talos_" + str(robot_id) + "/root_joint",
            "talos_" + str(robot_id) + "/root_joint",
            [0, 0, 1, 0, 0, 0, 1],
        )
        self.robots_locks.append("talos_" + str(robot_id) + "/root_joint")
        for name, value in joints_name_value_tuple:
            joint_name = "talos_" + str(robot_id) + "/" + name
            self.q[self.robot.rankInConfiguration[joint_name]] = value
            self.ps.createLockedJoint(joint_name, joint_name, [value])
            self.robots_locks.append(joint_name)

    def getRankAndConfigSize(self, robot_id):
        robot_name = "talos_" + str(robot_id)
        rank = self.robot.rankInConfiguration[robot_name + "/root_joint"]
        size = sum(
            [
                self.robot.getJointConfigSize(joint_name)
                for joint_name in calib.robot.getJointNames()
                if robot_name in joint_name
            ]
        )
        return rank, size

    def setGaze(self, robot_id, checkerboard_pose):
        robot_name = "talos_" + str(robot_id)
        position = (
            checkerboard_pose.position.x,
            checkerboard_pose.position.y,
            checkerboard_pose.position.z,
        )
        orientation = (
            checkerboard_pose.orientation.x,
            checkerboard_pose.orientation.y,
            checkerboard_pose.orientation.z,
            checkerboard_pose.orientation.w,
        )
        self.ps.createPositionConstraint(
            robot_name + "gaze",
            robot_name + "/calib_rgb_joint",
            robot_name + "/calib_mire_joint_2",
            position,
            (0, 0, 0),
            (True, True, True),
        )
        self.ps.createOrientationConstraint(
            robot_name + "gaze_O",
            robot_name + "/calib_mire_joint_2",
            robot_name + "/calib_rgb_joint",
            orientation,
            [True, True, True],
        )
        self.robots_gaze.append(robot_name + "gaze")
        self.robots_gaze.append(robot_name + "gaze_O")

    def constrainFreeflyers(self):
        for robot_id in range(1, len(self.robots)):
            robot_name = "talos_" + str(robot_id)
            self.client.basic.problem.createIdentityConstraint(
                robot_name + "_id_rgb",
                ["talos_0/calib_rgb_joint"],
                [robot_name + "/calib_rgb_joint"],
            )
            self.client.basic.problem.createIdentityConstraint(
                robot_name + "_id_mire",
                ["talos_0/calib_mire_joint_2"],
                [robot_name + "/calib_mire_joint_2"],
            )
            self.robots_identity_constraints.append(robot_name + "_id_rgb")
            self.robots_identity_constraints.append(robot_name + "_id_mire")

    def optimize(self, q_init, robots_id):
        client.basic.problem.resetConstraints()
        calib.robot.setCurrentConfig(q_init)

        client.basic.problem.addLockedJointConstraints("unused", calib.robots_locks)
        gaze = [
            c
            for c in calib.robots_gaze
            if any(["talos_" + str(robot_id) in c for robot_id in robots_id])
        ]
        num_constraints = gaze + calib.robots_identity_constraints
        client.basic.problem.addNumericalConstraints(
            "unused", num_constraints, [0 for _ in num_constraints]
        )

        projOk, q_proj, error = client.basic.problem.applyConstraints(q_init)
        if projOk:
            # optOk, q_estimated, error = client.basic.problem.optimize (q_proj)
            optOk, q_proj, error = client.basic.problem.optimize(q_proj)
            if optOk:
                print("All was good!")
            else:
                print("Optimisation error: " + str(error))
        else:
            print("Projection error: " + str(error))

        rank_rgb = calib.robot.rankInConfiguration["talos_0/calib_rgb_joint"]
        return q_proj, q_proj[rank_rgb : rank_rgb + 7], error


def openBag(path, calib, robot_id):
    bag = rosbag.Bag(path)

    joint_states = bag.read_messages(topics=["/joint_states"]).next().message
    joints_name_value_tuple = zip(joint_states.name, joint_states.position)

    checkerboard_pose = (
        bag.read_messages(topics=["/checkerdetector/objectdetection_pose"])
        .next()
        .message.pose
    )

    # robot_id = calib.initRobot()
    calib.setLockJoints(robot_id, joints_name_value_tuple)

    randlim = 0.02
    if robot_id in []:
        randlim = 0.5
    else:
        randlim = 0.02

    calib.q[robot_id * 65 + 28 : robot_id * 65 + 28 + 7] = [
        0.05 + random.gauss(0.0, randlim),
        0.03 + random.gauss(0.0, randlim),
        0.015 + random.gauss(0.0, randlim),
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    calib.q[robot_id * 65 + 58 : robot_id * 65 + 58 + 7] = [
        0.02,
        -0.01,
        -0.025,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    calib.robot.setCurrentConfig(calib.q)
    rgb_pose = Transform(
        calib.robot.getJointPosition("talos_" + str(robot_id) + "/calib_rgb_joint")
    )
    mire_pose = Transform(
        calib.robot.getJointPosition("talos_" + str(robot_id) + "/calib_mire_joint_2")
    )
    t = (rgb_pose.inverse() * mire_pose).toTuple()
    checkerboard_pose.position.x = t[0]
    checkerboard_pose.position.y = t[1]
    checkerboard_pose.position.z = t[2]
    checkerboard_pose.orientation.x = t[3]
    checkerboard_pose.orientation.y = t[4]
    checkerboard_pose.orientation.z = t[5]
    checkerboard_pose.orientation.w = t[6]
    calib.q[robot_id * 65 + 28 : robot_id * 65 + 28 + 7] = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    calib.q[robot_id * 65 + 58 : robot_id * 65 + 58 + 7] = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    calib.robot.setCurrentConfig(calib.q)

    calib.setGaze(robot_id, checkerboard_pose)

    bag.close()
    return checkerboard_pose


calib = Calibration(client)

calib.client.basic.problem.setDefaultLineSearchType("Backtracking")

for i in range(1, 22):
    print(i)
    calib.initRobot()

# Add constraints
for i in range(1, 22):
    openBag("/usr/localDev/rosbag-calib/" + str(i) + ".bag", calib, i - 1)
calib.constrainFreeflyers()

q_init = calib.robot.getCurrentConfig()

# -----
# RANSAC
# -----
best_rgb_pose = []
best_sample = []
best_error = 1e10
best_proj = []

max_nb_tries = 100
for i in range(max_nb_tries):
    sample = random.sample(range(len(calib.robots)), 15)

    q_proj, rgb_pose, error = calib.optimize(q_init, sample)

    print(
        "Real error: "
        + str(
            math.sqrt(
                (rgb_pose[0] - 0.02) ** 2
                + (rgb_pose[1] + 0.01) ** 2
                + (rgb_pose[2] + 0.025) ** 2
            )
        )
        + " vs error proj: "
        + str(error)
    )
    print("Pose: " + str(rgb_pose))

print("Results:")
print("Error: " + str(best_error))
print("Sample: " + str(best_sample))
print("Pose: " + str(best_rgb_pose))
