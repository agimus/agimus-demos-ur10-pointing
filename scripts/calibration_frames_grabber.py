#!/usr/bin/env python2.7
import argparse
import sys

import rospy
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty, EmptyResponse


parser = argparse.ArgumentParser()

parser.add_argument("world_frame", help="Name of the origin frame")
parser.add_argument("end_effector_frame", help="Name of the end effector frame")
parser.add_argument("camera_frame", help="Name of the camera frame")
parser.add_argument("object_frame", help="Name of the object frame")
args = parser.parse_args(sys.argv[1:5])

publish = False
seq = 0


# Get wMe and cMo from TF, send them to another topic
def republisher(pub):
    publish = True
    return EmptyResponse()


if __name__ == "__main__":
    pub = rospy.Publisher("calibration_frames", TransformStamped, queue_size=10)
    rospy.init_node("calibration_frames_grabber", anonymous=True)
    repub = rospy.Service("republish", Empty, republisher)

    listener = tf.TransformListener()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if publish:
            try:
                (wMeTrans, wMeRot) = listener.lookupTransform(
                    args.end_effector_frame, args.world_frame, rospy.Time(0)
                )
                (cMoTrans, cMoRot) = listener.lookupTransform(
                    args.object_frame, args.camera_frame, rospy.Time(0)
                )
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                print("Error while looking for cMo and wMe transforms")
                continue

            stamp = rospy.Time.now()

            cMo = TransformStamped(
                header=Header(seq, stamp, args.camera_frame),
                child_frame_id=args.object_frame,
                transform=Transform(cMoTrans, cMoRot),
            )
            wMe = TransformStamped(
                header=Header(seq, stamp, args.world_frame),
                child_frame_id=args.end_effector_frame,
                transform=Transform(wMeTrans, wMeRot),
            )

            pub.publish(cMo)
            pub.publish(wMe)
            publish = False

        rate.sleep()
