from math import sqrt
from  geometry_msgs.msg import PoseWithCovarianceStamped
import rospy

rospy.init_node("localization_initial_pose")
pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 1)
rate = rospy.Rate(1) # 1hz

p  = PoseWithCovarianceStamped()
p.header.seq = 1
p.header.stamp.secs = int(rospy.get_time())
p.header.stamp.nsecs = 0
p.header.frame_id = 'map'
p.pose.pose.position.x = -2.94
p.pose.pose.position.y = 4.114
p.pose.pose.position.z = 0.098
p.pose.pose.orientation.x = 0
p.pose.pose.orientation.y = 0
p.pose.pose.orientation.z = -sqrt(2)/2
p.pose.pose.orientation.w = sqrt(2)/2
p.pose.covariance = [.01, 0, 0, 0, 0, 0,
                     0, .01, 0, 0, 0, 0,
                     0, 0, .001, 0, 0, 0,
                     0, 0, 0, .001, 0, 0,
                     0, 0, 0, 0, 0.001, 0,
                     0, 0, 0, 0, 0, .01]
try:
    while not rospy.is_shutdown():
        p.header.stamp.secs = int(rospy.get_time())
        pub.publish(p)
        p.header.seq += 1
        rate.sleep()
except rospy.ROSInterruptException:
    pass

    
