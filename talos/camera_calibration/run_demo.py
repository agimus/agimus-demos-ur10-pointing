import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import UInt32

script = "rosbag-calib.sh"

rospy.init_node('demo_camera_calibration')

rospy.wait_for_service ('/agimus/status')
get_status = rospy.ServiceProxy ('/agimus/status', Trigger)
start_path = rospy.Publisher ('/agimus/start_path', UInt32)

def wait_for_agimus (ready):
    rate = rospy.Rate(10)
    rsp = get_status ()
    while ready == rsp.success:
        rate.sleep()
        rsp = get_status ()

def acquire_image ():
    # TODO use rosbag Python API
    import os
    os.system (rosbag)

def run_demo (hpppaths):
    img = [ True, ] * len(hpppaths)
    img[-1] = False
    for id, save_topics in zip(hpppaths, img):
        wait_for_agimus(ready=True)
        start_path (id)
        wait_for_agimus(ready=False)
        wait_for_agimus(ready=True)
        if save_topics:
            acquire_image()
