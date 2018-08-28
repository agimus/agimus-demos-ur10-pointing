#!/usr/bin/env python2.7
from agimus_vision.srv import AddAprilTagService
import rospy, argparse, sys

parser = argparse.ArgumentParser()

parser.add_argument("id", help="Id of the tag")
parser.add_argument("size", help="Size of the tag in mm")
parser.add_argument("node_name", help="Name of child frame in tf tree")
args = parser.parse_args(sys.argv[1:4])

if __name__ == "__main__":
    rospy.init_node ("visual_tag_initializer", anonymous=True)
    ns = rospy.get_namespace()
    rospy.wait_for_service (ns+"add_april_tag_detector")

    add_april_tag_detector = rospy.ServiceProxy (ns+"add_april_tag_detector", AddAprilTagService)
    add_april_tag_detector (int(args.id), float(args.size), str(args.node_name))
