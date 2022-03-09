#!/usr/bin/env python  
import rospy
import math
import tf
from realsense2_camera.msg import Extrinsics
from scipy.spatial.transform import Rotation as R
import numpy as np

def run():
    rospy.init_node('publish_camera_extrinsics')

    listener = tf.TransformListener()

    publisher = rospy.Publisher('/camera/extrinsics/depth_to_color', Extrinsics, queue_size=1)

    rate = rospy.Rate(1.0)
    counter = 1
    while not rospy.is_shutdown():
        try:
            pose = listener.lookupTransform('/camera_color_optical_frame', '/camera_depth_optical_frame', rospy.Time(0))
            (trans,rot) = pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        ext_msg = Extrinsics()
        ext_msg.header.seq = counter
        ext_msg.header.stamp = rospy.Time.now()
        ext_msg.header.frame_id = ""
        ext_msg.translation = trans

        rot = R.from_quat(rot)
        ext_msg.rotation = list(rot.as_dcm().flatten())

        publisher.publish(ext_msg)
        counter += 1
        rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
