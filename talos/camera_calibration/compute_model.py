# /usr/bin/env python

import random

import rosbag
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
            self.ps.setMaxIterProjection(100)

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
        root_joint_rank = self.robot.rankInConfiguration[
            "talos_" + str(robot_id) + "/root_joint"
        ]
        self.q[root_joint_rank : root_joint_rank + 7] = [0, 0, 1, 0, 0, 0, 1]
        self.robots_locks.append("talos_" + str(robot_id) + "/root_joint")
        # self.ps.createLockedJoint("talos_" + str(robot_id) + "/calib_mire_joint_2", "talos_" + str(robot_id) + "/calib_mire_joint_2", [0, 0, 0, 0, 0, 0, 1])
        # mire_joint_rank = self.robot.rankInConfiguration["talos_" + str(robot_id) + "/calib_mire_joint_2"]
        # self.q[mire_joint_rank:mire_joint_rank + 7] = [0, 0, 0, 0, 0, 0, 1]
        # self.robots_locks.append("talos_" + str(robot_id) + "/calib_mire_joint_2")
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
            [True, True, True],
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
        self.q_proj = self.q
        client.basic.problem.resetConstraints()
        calib.robot.setCurrentConfig(q_init)

        client.basic.problem.addLockedJointConstraints("unused1", calib.robots_locks)
        gaze = [
            c
            for c in calib.robots_gaze
            if any(["talos_" + str(robot_id) in c for robot_id in robots_id])
        ]
        num_constraints = gaze + calib.robots_identity_constraints
        client.basic.problem.addNumericalConstraints(
            "unused2", num_constraints, [0 for _ in num_constraints]
        )
        # client.basic.problem.addNumericalConstraints("mighty_cam_cost", ["cam_cost"], [ 1 ])
        # calib.client.basic.problem.setNumericalConstraintsLastPriorityOptional(True)

        projOk, self.q_proj, error = client.basic.problem.applyConstraints(q_init)
        if error < 1.0:
            optOk, self.q_proj, error = client.basic.problem.optimize(self.q_proj)
            if optOk:
                print("All was good! " + str(max(error)))
            else:
                print("Optimisation error: " + str(max(error)))
        else:
            print("Projection error: " + str(error))

        rank_rgb = calib.robot.rankInConfiguration["talos_0/calib_rgb_joint"]
        rank_mire = calib.robot.rankInConfiguration["talos_0/calib_mire_joint_2"]
        return (
            self.q_proj,
            self.q_proj[rank_rgb : rank_rgb + 7],
            max(error),
            self.q_proj[rank_mire : rank_mire + 7],
        )


def openBags(path, calib, robot_id):
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
    calib.setGaze(robot_id, checkerboard_pose)

    bag.close()
    return checkerboard_pose


def openBag(path, calib):
    bag = rosbag.Bag(path)
    robot_id = 0

    for _ in range(len(list(bag.read_messages(topics=["joints"])))):
        calib.initRobot()

    for (_, joint_states, _), (_, checkerboard_pose, _) in zip(
        bag.read_messages(topics=["joints"]), bag.read_messages(topics=["chessboard"])
    ):
        joints_name_value_tuple = zip(joint_states.name, joint_states.position)

        calib.setLockJoints(robot_id, joints_name_value_tuple)
        calib.setGaze(robot_id, checkerboard_pose.pose)
        robot_id += 1


calib = Calibration(client)

openBag("/usr/localDev/rosbag-calib/pyrene-calib.bag", calib)
calib.client.basic.problem.createPositionConstraint(
    "cam_cost",
    "talos_0/rgbd_rgb_optical_frame",
    "talos_0/calib_rgb_joint",
    (0, 0, 0),
    (0, 0, 0),
    (True, True, True),
)
calib.client.basic.problem.createOrientationConstraint(
    "cam_cost_O",
    "talos_0/rgbd_rgb_optical_frame",
    "talos_0/calib_rgb_joint",
    (0, 0, 0, 1),
    [True, True, True],
)

calib.constrainFreeflyers()

q_init = calib.robot.getCurrentConfig()

# -----
# RANSAC
# -----
best_rgb_pose = []
best_sample = []
best_error = 1e50
best_proj = []

max_nb_tries = 10
for i in range(max_nb_tries):
    sample = random.sample(range(len(calib.robots)), 10)

    q_proj, rgb_pose, error, mire_pose = calib.optimize(q_init, sample)
    print("Pose: " + str(rgb_pose))
    print("Pose mire: " + str(mire_pose))

    if error > 10.0:
        continue

    for robot_id in list(set(range(len(calib.robots))) - set(sample)):
        new_sample = sample + [robot_id]
        new_q_proj, new_rgb_pose, new_error, mire_pose = calib.optimize(
            q_proj, new_sample
        )

        if new_error < error:
            q_proj = new_q_proj
            rgb_pose = new_rgb_pose
            sample = new_sample
            error = new_error
            print("Pose: " + str(rgb_pose))

    if error < best_error:
        best_rgb_pose = rgb_pose
        best_sample = sample
        best_error = error
        best_proj = q_proj

print("Results:")
print("Error: " + str(best_error))
print("Sample: " + str(best_sample))
print("Pose: " + str(best_rgb_pose))
