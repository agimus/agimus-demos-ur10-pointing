#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_srvs.srv import Empty

rospy.init_node("publish_initial_pose", anonymous=True)
rospy.wait_for_service("/global_localization")
rospy.wait_for_service("/move_base/clear_costmaps")

clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

rootJointPose = rospy.get_param ("/robot_initial_pose", "0 0 0 0 0 0 1")
x, y, z, X, Y, Z, W = map (float, rootJointPose.split (' '))

msg = PoseWithCovarianceStamped()
msg.pose.pose.position = Point(x,y,z)
msg.pose.pose.orientation = Quaternion(X,Y,Z,W)
msg.pose.covariance = [ 0. ] * 36
msg.pose.covariance[0] = 0.01 # Covariance xx
msg.pose.covariance[6+1] = 0.01 # Covariance yy
msg.pose.covariance[-1] = 0.02 # Covariance Yaw-Yaw
desired_pose = msg.pose.pose

def equal(a, b, attrs, thr):
    for attr in attrs:
        if abs(getattr(a,attr)-getattr(b,attr)) >= thr:
            return False
    return True

rate = rospy.Rate(2)
init_pose_set = False
while not init_pose_set:
    pub.publish(msg)
    rate.sleep()
    current_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped).pose.pose
    init_pose_set = equal(current_pose.position, desired_pose.position, 'xyz', 0.01) \
            and equal(current_pose.orientation, desired_pose.orientation, 'xyzw', 0.02)
clear_costmaps()
