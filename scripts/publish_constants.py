#!/usr/bin/env python

import rospy, yaml, argparse
from sensor_msgs.msg import CameraInfo

parser = argparse.ArgumentParser()
parser.add_argument("camera_info", help="Yaml file containing the camera info.")
parser.add_argument("topic", help="Published topic")
parser.add_argument("frame", help="Frame the camera is attached to")
parser.add_argument("--rate", help="Change frequency of publication.", type=int, default=30)

args = parser.parse_args(rospy.myargv()[1:])

def run():
    rospy.init_node('publish_constant', anonymous=True)
    pub = rospy.Publisher(args.topic, CameraInfo, queue_size=10)
    rate = rospy.Rate(30)

    filename = args.camera_info
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

    camera_info.header.frame_id = args.frame
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
