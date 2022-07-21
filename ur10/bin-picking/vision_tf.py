# Transform a given input pose from one fixed frame to another
import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs  #    poseStamped  >> pose --header


def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame   #Frame this data is associated with
    pose_stamped.header.stamp = rospy.Time(0)    # return the latest available data for a specific transform

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


# Test Case
rospy.init_node("transform_test")

my_pose = Pose()
my_pose.position.x = -0.25
my_pose.position.y = -0.50
my_pose.position.z = +1.50
my_pose.orientation.x = 0.634277921154
my_pose.orientation.y = 0.597354098852
my_pose.orientation.z = 0.333048372508
my_pose.orientation.w = 0.360469667089

transformed_pose = transform_pose(my_pose, "wrist_3_link", "base_link")

print(transformed_pose)
