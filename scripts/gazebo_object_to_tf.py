#!/usr/bin/env python2.7
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
import tf, rospy, argparse, sys

parser = argparse.ArgumentParser()
parser.add_argument("object_name", help="Name of the object")
parser.add_argument("link_name", help="Link of the object")
parser.add_argument("world_name", help="Name of the parent link")
args = parser.parse_args(sys.argv[1:4])

gz_name = "::".join([args.object_name, args.link_name])
tf_name = "/".join([args.object_name, args.link_name])
world_name = args.world_name

if __name__ == "__main__":
    rospy.init_node ("gazebo_object_to_tf")
    rate = rospy.Rate (10)
    rospy.wait_for_service ("/gazebo/get_link_state")
    get_link_state = rospy.ServiceProxy ("/gazebo/get_link_state", GetLinkState)
    request = GetLinkStateRequest (gz_name, "")
    tb = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        rospy.wait_for_service ("/gazebo/get_link_state")
        ans = get_link_state(request)
        if ans.success:
            tb.sendTransform ((ans.link_state.pose.position.x,
                               ans.link_state.pose.position.y,
                               ans.link_state.pose.position.z,),
                              (ans.link_state.pose.orientation.x,
                               ans.link_state.pose.orientation.y,
                               ans.link_state.pose.orientation.z,
                               ans.link_state.pose.orientation.w,),
                              rospy.Time.now(),
                              tf_name,
                              world_name)
        else:
            rospy.logerr (ans.status_message)
        rate.sleep()
