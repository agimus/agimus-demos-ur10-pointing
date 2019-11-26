#!/usr/bin/env python

import sys
import rospy
import yaml
from sensor_msgs.msg import CameraInfo

def run():
    rospy.init_node('publish_constant', anonymous=True)
    pub = rospy.Publisher('/rgbd/rgb/high_res/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(30)

    filename = sys.argv [1]
    with open(filename, 'r') as f:
        camera_data = yaml.load(f)

    camera_info = CameraInfo()
    camera_info.height = camera_data['height']
    camera_info.width = camera_data['width']
    camera_info.K = camera_data['K']
    camera_info.D = camera_data['D']
    camera_info.R = camera_data['R']
    camera_info.P = camera_data['P']
    camera_info.distortion_model = camera_data['distortion_model']

    camera_info.header.frame_id = 'rgbd_rgb_optical_frame'
    # Setting this to zero means 'take the newest camera pose if TF'.
    camera_info.header.stamp = rospy.Time(0)

    while not rospy.is_shutdown():
        pub.publish(camera_info)
        rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
