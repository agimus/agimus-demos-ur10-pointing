#!/usr/bin/env python  
import rospy
import math
import tf
#import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

def run():
    rospy.init_node('publish_wmc')

    listener = tf.TransformListener()

    wmc = rospy.Publisher('/pose', PoseStamped, queue_size=1)

    rate = rospy.Rate(30.0)
    counter = 1
    while not rospy.is_shutdown():
        try:
            pose = listener.lookupTransform('ref_camera_link', 'world', rospy.Time(0))
            (trans,rot) = pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        pose_msg = PoseStamped()
        pose_msg.header.seq = counter
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = ""
        pose_msg.pose.position.x = trans[0]
        pose_msg.pose.position.y = trans[1]
        pose_msg.pose.position.z = trans[2]
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]

        wmc.publish(pose_msg)
        counter += 1
        rate.sleep()


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
