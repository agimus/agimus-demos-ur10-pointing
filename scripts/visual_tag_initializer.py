#!/usr/bin/env python2.7
import argparse

import rospy
from agimus_hpp.ros_tools import wait_for_service
from agimus_vision.srv import AddAprilTagService

parser = argparse.ArgumentParser()

parser.add_argument("id", help="Id of the tag")
parser.add_argument("size", help="Size of the tag in mm")
parser.add_argument("node_name", help="Name of child frame in tf tree")
parser.add_argument("parent_node_name", help="Name of parent frame in tf tree")

args = parser.parse_args(rospy.myargv()[1:])

def add_april_tag ():
    rospy.init_node("visual_tag_initializer", anonymous=True)
    wait_for_service("add_april_tag_detector")

    add_april_tag_detector = rospy.ServiceProxy(
        "add_april_tag_detector", AddAprilTagService
    )
    add_april_tag_detector(int(args.id), float(args.size),
            str(args.node_name), str(args.parent_node_name))

    rospy.loginfo(
        "Added APRIL tag {0}, size {1}mm, {2}, {3}".format(
            args.id, args.size, args.node_name, args.parent_node_name
        )
    )

if __name__ == "__main__":
    try:
        add_april_tag()
    except rospy.ROSInterruptException:
        pass
