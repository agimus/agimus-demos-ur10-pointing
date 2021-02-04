#!/usr/bin/env python2.7
import rospy, time
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_srvs.srv import Empty

def attach_driller_to_tiago_gripper():
    from gazebo_ros_link_attacher.srv import Attach, AttachRequest
    from gazebo_msgs.srv import GetModelState, GetModelStateRequest

    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    get_model_state.wait_for_service()
    def wait_for_model(name):
        rospy.loginfo("Waiting for " + name + " model to be spawned.")
        ok = False
        req = GetModelStateRequest(name, '')
        while not ok:
            rsp = get_model_state(req)
            ok = rsp.success
            time.sleep(1)
    wait_for_model("tiago")
    wait_for_model("driller")

    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    req = AttachRequest()
    req.model_name_1 = "tiago"
    req.link_name_1 = "arm_7_link"
    req.model_name_2 = "driller"
    req.link_name_2 = "base_link"

    res = attach_srv.call(req)
    if not res.ok:
        rospy.logerr("Could not attach driller to tiago hand.")

def publish_initial_pose():
    rospy.wait_for_service("/global_localization")

    clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
    clear_costmaps.wait_for_service()

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

rospy.init_node("publish_initial_pose", anonymous=True)

attach_driller_to_tiago_gripper()
publish_initial_pose()
