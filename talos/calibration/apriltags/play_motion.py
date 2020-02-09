#!/usr/bin/env python
# Copyright 2020 CNRS - Airbus SAS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from csv import writer
import roslib
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState
from dynamic_graph_bridge_msgs.msg import Vector
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from smach_msgs.msg import SmachContainerStatus
from std_msgs.msg import Bool, UInt32
from hpp.corbaserver import Client as HppClient

## Control calibration motions on Talos robot
#
#  iteratively
#    - trigger motions planned by HPP by publishing in "/agimus/start_path"
#      topic,
#    - wait for end of motion by listening "/agimus/status/running" topic,
#    - record tf transformation between camera (topic ) and gripper
#    - record joint values (topic ).
class CalibrationControl (object):
    cameraFrame = "rgbd_rgb_optical_frame"
    leftGripper  = "gripper_left_base_link_measured"
    rightGripper = "gripper_right_base_link_measured"
    maxDelay = rospy.Duration (1,0)
    def __init__ (self) :
        rospy.init_node ('calibration_control')
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration (1,0))
        self.tf2Listener = tf2_ros.TransformListener(self.tfBuffer)
        self.pubStartPath = rospy.Publisher ("/agimus/start_path", UInt32,
                                             queue_size=1)
        self.subRunning = rospy.Subscriber ("/agimus/status/running", Bool,
                                            self.runningCallback)
        self.subStatus = rospy.Subscriber \
                         ("/agimus/agimus/smach/container_status",
                          SmachContainerStatus, self.statusCallback)
        self.subSotJointStates = rospy.Subscriber ("/agimus/sot/state", Vector,
                                                   self.sotJointStateCallback)
        self.subRosJointState = rospy.Subscriber ("/jointStates", JointState,
                                                  self.rosJointStateCallback)
        self.running = False
        self.sotJointStates = None
        self.rosJointStates = None
        self.jointNames = None
        self.pathId = 0
        self.hppClient = HppClient ()
        self.count = 0
        self.measures = list ()

    def playPath (self, pathId):
        nbPaths = self.hppClient.problem.numberPaths ()
        if pathId >= nbPaths:
            raise RuntimeError ("pathId ({}) is bigger than number paths {}"
                                .format (pathId, nbPaths - 1))
        self.errorOccured = False
        self.pubStartPath.publish (pathId)
        self.waitForEndOfMotion ()
        if not self.errorOccured:
            self.collectData ()

    def collectData (self):
        measure = dict ()
        # record position of left gripper
        try:
            self.leftGripperInCamera = self.tfBuffer.lookup_transform\
                (self.cameraFrame, self.leftGripper, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.leftGripperInCamera = None
            rospy.loginfo ("No data for left gripper")
        # record position of right gripper
        try:
            self.rightGripperInCamera = self.tfBuffer.lookup_transform\
                (self.cameraFrame, self.rightGripper, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.rightGripperInCamera = None
            rospy.loginfo ("No data for right gripper")
        now = rospy.Time.now ()
        # Get position of left gripper in camera
        if self.leftGripperInCamera:
            t = self.leftGripperInCamera.header.stamp
            # Check that data is recent enough
            if abs (now - t) < self.maxDelay:
                measure ["left_gripper"] = self.leftGripperInCamera.transform
            else:
                rospy.loginfo ("time left gripper from now: {}".
                               format ((now - t).secs + 1e-9*(now - t).nsecs))
        # Get position of right gripper in camera
        if self.rightGripperInCamera:
            t = self.rightGripperInCamera.header.stamp
            # Check that data is recent enough
            if abs (now - t) < self.maxDelay:
                measure ["right_gripper"] = \
                    self.rightGripperInCamera.transform
            else:
                rospy.loginfo ("time right gripper from now: {}".
                               format ((now - t).secs + 1e-9*(now - t).nsecs))
        # Get joint values
        if self.rosJointStates:
            measure ["joint_states"] = self.rosJointStates
        elif self.sotJointStates:
            measure ["joint_states"] = self.sotJointStates
        self.measures.append (measure)

    def save (self, filename):
        with open (filename, "w") as f:
            # Write joint names
            if self.jointNames:
                w = writer (f)
                w.writerow (['joint_names'] + self.jointNames)
            # write measures
            for measure in self.measures:
                if not measure.has_key ("left_gripper") and \
                   not measure.has_key ("right_gripper"):
                    continue
                line = ""
                # joint_states
                assert measure.has_key ("joint_states")
                line += "joint_states,"
                for jv in measure ["joint_states"]:
                    line += "{},".format (jv)
                # grippers
                for k in ["left_gripper", "right_gripper"]:
                    if measure.has_key (k):
                        line += "{},".format (k)
                        T = measure [k].translation
                        R = measure [k].rotation
                        line += "{},{},{},{},{},{},{},".format\
                        (T.x,T.y,T.z,R.x,R.y,R.z,R.w)
                line = line [:-1] + "\n"
                f.write (line)

    def waitForEndOfMotion (self):
        rate = rospy.Rate(2) # 2hz
        # wait for motion to start
        rospy.loginfo ("wait for motion to start")
        while not self.running:
            rate.sleep ()
            if rospy.is_shutdown (): raise RuntimeError ("rospy shutdown.")
        # wait for motion to end
        rospy.loginfo ("wait for motion to end")
        while self.running:
            rate.sleep ()
            if rospy.is_shutdown (): raise RuntimeError ("rospy shutdown.")
        # wait for one more second
        rospy.loginfo ("motion stopped")
        for i in range (2):
            rate.sleep ()

    def runningCallback (self, msg):
        self.running = msg.data

    def statusCallback (self, msg):
        if msg.active_states [0] == 'Error':
            self.errorOccured = True
            rospy.loginfo ('Error occured.')

    def sotJointStateCallback (self, msg):
        self.sotJointStates = msg.data

    def rosJointStateCallback (self, msg):
        self.rosJointStates = msg.position
        if not self.jointNames:
            self.jointNames = msg.name

if __name__ == '__main__':
    cc = CalibrationControl ()
    i=0
    nbPaths = cc.hppClient.problem.numberPaths ()
    while i < nbPaths - 1:
        cc.playPath (i)
        if not cc.errorOccured:
            i+=1
        rospy.sleep (1)
