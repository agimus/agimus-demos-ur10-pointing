#!/usr/bin/env python2.7
import argparse
import sys

import rospy
from agimus_hpp.ros_tools import wait_for_service
from agimus_vision.srv import AddAprilTagService

parser = argparse.ArgumentParser()

parser.add_argument("id", help="Id of the tag")
parser.add_argument("size", help="Size of the tag in mm")
parser.add_argument("node_name", help="Name of child frame in tf tree")
args = parser.parse_args(sys.argv[1:4])

if __name__ == "__main__":
    rospy.init_node("visual_tag_initializer", anonymous=True)
    wait_for_service("add_april_tag_detector")

    add_april_tag_detector = rospy.ServiceProxy(
        "add_april_tag_detector", AddAprilTagService
    )
    add_april_tag_detector(int(args.id), float(args.size), str(args.node_name))

    rospy.loginfo(
        "Added APRIL tag {0}, size {1}mm, {2}".format(
            args.id, args.size, args.node_name
        )
    )
