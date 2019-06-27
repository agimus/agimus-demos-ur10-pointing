# /usr/bin/env python

import time

import rosbag
import rospy
from geometry_msgs.msg import Transform as TransformROS
from hpp import Transform
from hpp.corbaserver.manipulation.robot import CorbaClient, Robot


def publishTransform(publisher, transform):
    t = TransformROS()
    t.rotation.x = transform.quaternion.array[0]
    t.rotation.y = transform.quaternion.array[1]
    t.rotation.z = transform.quaternion.array[2]
    t.rotation.w = transform.quaternion.array[3]
    t.translation.x = transform.translation[0]
    t.translation.y = transform.translation[1]
    t.translation.z = transform.translation[2]

    publisher.publish(t)


client = CorbaClient()
bag = rosbag.Bag("/usr/localDev/rosbag-calib/pyrene-calib.bag")

Robot.packageName = "agimus_demos"
Robot.urdfName = "talos"
Robot.urdfSuffix = "_calibration_camera"
Robot.srdfSuffix = ""

robot = Robot("talos", "talos", rootJointType="freeflyer", client=client)
q = robot.getCurrentConfig()

pub_cMo = rospy.Publisher("/camera_object", TransformROS, queue_size=100)
pub_fMe = rospy.Publisher("/world_effector", TransformROS, queue_size=100)
rospy.init_node("pose_sender", anonymous=True)

i = 0
for (_, joint_states, _), (_, checkerboard_pose, _) in zip(
    bag.read_messages(topics=["joints"]), bag.read_messages(topics=["chessboard"])
):

    root_joint_rank = robot.rankInConfiguration["talos/root_joint"]
    q[root_joint_rank : root_joint_rank + 7] = [0, 0, 1, 0, 0, 0, 1]

    joints_name_value_tuple = zip(joint_states.name, joint_states.position)
    for name, value in joints_name_value_tuple:
        joint_name = "talos/" + name
        q[robot.rankInConfiguration[joint_name]] = value
    robot.setCurrentConfig(q)

    gripper = Transform(robot.getJointPosition("talos/gripper_left_base_link_joint"))
    camera = Transform(robot.getJointPosition("talos/rgbd_rgb_optical_joint"))
    fMe = gripper.inverse() * camera

    cMo = Transform(
        [
            checkerboard_pose.pose.position.x,
            checkerboard_pose.pose.position.y,
            checkerboard_pose.pose.position.z,
            checkerboard_pose.pose.orientation.x,
            checkerboard_pose.pose.orientation.y,
            checkerboard_pose.pose.orientation.z,
            checkerboard_pose.pose.orientation.w,
        ]
    )

    publishTransform(pub_cMo, cMo)
    publishTransform(pub_fMe, fMe)

    i += 1

    time.sleep(0.05)
