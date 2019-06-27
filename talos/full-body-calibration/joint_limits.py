#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from threading import Lock
from sensor_msgs.msg import JointState


class JointLimits(object):

    """Record the mechanical joint limits while the joints are manually moved"""

    def __init__(self):
        """TODO: to be defined1. """
        self.joint_limits = dict()
        self.state_lock = Lock()

        rospy.init_node('JointLimitsRecorder', anonymous=True)

        rospy.Subscriber('/joint_states', JointState, self.jointStateCallback)

    def jointStateCallback(self, data):
        """Receive the joint state updates and store the limits

        :data: JointState message

        """
        with self.state_lock:
            for joint_id, joint_name in enumerate(data.name):
                if joint_name in self.joint_limits:
                    self.joint_limits[joint_name] = {
                            'min' : min(data.position[joint_id], self.joint_limits[joint_name]['min']),
                            'max' : max(data.position[joint_id], self.joint_limits[joint_name]['max']) }
                else:
                    self.joint_limits[joint_name] = {
                            'min' : data.position[joint_id],
                            'max' : data.position[joint_id]}
                
    def save(self, filename):
        """Save the joint limits to a file

        :filename: Name of the file
        """
        import json

        json = json.dumps(self.joint_limits)
        with open(filename, 'w') as f:
            f.write(json)
