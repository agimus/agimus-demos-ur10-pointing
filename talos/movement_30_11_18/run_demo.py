import rosbag
import rospy
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger

rospy.init_node("demo_camera_calibration")

rospy.wait_for_service("/agimus/status")


class RecordCalib:
    from sensor_msgs.msg import JointState

    def __init__(self):
        from time import gmtime, strftime

        strdatetime = strftime("%d-%m-%Y %H:%M", gmtime())
        self.bag = rosbag.Bag("pyrene-calib-" + strdatetime + ".bag", "w")

        self.get_status = rospy.ServiceProxy("/agimus/status", Trigger)
        self.start_path = rospy.Publisher("/agimus/start_path", UInt32)

        rospy.Subscriber("/joint_states", self.jointStates)
        rospy.Subscriber("/checkerdetector/objectdetection_pose", self.chessboardPose)

    def jointStates(self, data):
        self.joints = data

    def chessboardPose(self, data):
        self.chessboard_pose = data

    def wait_for_agimus(self, ready):
        rate = rospy.Rate(10)
        rsp = self.get_status()
        while ready == rsp.success:
            rate.sleep()
            rsp = self.get_status()

    def acquire_image(self):
        # TODO: Check if timestamps from the joints and chessboard pose are ~same
        t1 = self.joints.data.header.stamp
        t2 = self.chessboard_pose.data.header.stamp

        if abs((t1.secs + t1.nsecs / 1e9) - (t2.secs + t2.nsecs / 1e9)) < 1.0:
            self.bag.write("joints", self.joints.data)
            self.bag.write("chessboard", self.chessboard_pose.data)

    def run_demo(self, hpppaths):
        img = [True] * len(hpppaths)
        img[-1] = False
        for id, save_topics in zip(hpppaths, img):
            self.wait_for_agimus(ready=True)
            self.start_path(id)
            self.wait_for_agimus(ready=False)
            self.wait_for_agimus(ready=True)

            rate = rospy.Rate(25)
            for i in range(10):
                rospy.spinOnce()
                rate.sleep()

            if save_topics:
                self.acquire_image()

    def close(self):
        self.bag.close()
