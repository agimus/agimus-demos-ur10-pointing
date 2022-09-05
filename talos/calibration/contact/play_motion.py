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
from std_msgs.msg import Bool, UInt32, String
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
        self.subRosJointState = rospy.Subscriber ("/joint_states", JointState,
                                                  self.rosJointStateCallback)
        self.subStatusDescription = rospy.Subscriber\
            ("/agimus/status/description", String, self.statusDescriptionCallback)
        self.running = False
        self.sotJointStates = None
        self.rosJointStates = None
        self.jointNames = None
        self.pathId = 0
        self.hppClient = HppClient ()
        self.count = 0
        self.measurements = list ()

    def playPath (self, pathId):
        nbPaths = self.hppClient.problem.numberPaths ()
        if pathId >= nbPaths:
            raise RuntimeError ("pathId ({}) is bigger than number paths {}"
                                .format (pathId, nbPaths - 1))
        self.errorOccured = False
        self.pubStartPath.publish (pathId)
        self.waitForEndOfMotion ()

    def collectData (self, gripper, handle):
        measurement = dict ()
        measurement["gripper"] = gripper
        measurement["handle"] = handle
        # Get joint values
        if self.rosJointStates:
            measurement ["joint_states"] = self.rosJointStates
        elif self.sotJointStates:
            measurement ["joint_states"] = self.sotJointStates
        self.measurements.append (measurement)

    def save (self, filename):
        with open (filename, "w") as f:
            # Write joint names
            if self.jointNames:
                w = writer (f)
                w.writerow (['joint_names'] + self.jointNames)
            # write measurements
            for measurement in self.measurements:
                line = ""
                # gripper
                line += "gripper,"
                line += measurement["gripper"]
                # handle
                line += ",handle,"
                line += measurement["handle"]
                # joint_states
                assert measurement.has_key ("joint_states")
                line += ",joint_states,"
                for jv in measurement ["joint_states"]:
                    line += "{},".format (jv)
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

    def statusDescriptionCallback(self, msg):
        text = msg.data
        if text[:22] != "Executing post-action ": return
        text = text[22:]
        i = text.find(' < ')
        if i == -1: return
        gripper = text[:i]
        text = text[i+3:]
        i = text.find(' | ')
        if i == -1: return
        handle = text[:i]
        text = text[i+3:]
        if text.find('_21') != -1:
            self.collectData(gripper, handle)
                     
def playAllPaths (startIndex):
    i = startIndex
    while i < nbPaths - 1:
        cc.playPath (i)
        if not cc.errorOccured:
            print("Ran {}".format(i))
            i+=1
        rospy.sleep (1)

if __name__ == '__main__':
    cc = CalibrationControl ()
    nbPaths = cc.hppClient.problem.numberPaths ()
    # playAllPaths (0)
