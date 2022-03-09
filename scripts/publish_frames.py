#!/usr/bin/env python  
import rospy
import math
import tf
#import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

def run():
    rospy.init_node('publish_frames')

    listener = tf.TransformListener()
    publisher = tf.TransformBroadcaster()

    rate = rospy.Rate(30.0)
    counter = 1
    while not rospy.is_shutdown():
        try:
            translation, rotation = listener.lookupTransform\
                ('part/plaque_link', 'part/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            continue

        publisher.sendTransform(translation, rotation, rospy.Time.now(),
                                'part/base_link_measured',
                                'part/plaque_link_smoothed')
        rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
