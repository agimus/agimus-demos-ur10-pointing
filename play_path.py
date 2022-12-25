#!/usr/bin/env python
# Copyright 2021 CNRS - Airbus SAS
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
import time, roslib
import rospy
import math
import tf2_ros
import cv2 as cv
import eigenpy, numpy as np
from sensor_msgs.msg import Image, CameraInfo
from dynamic_graph_bridge_msgs.msg import Vector
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from smach_msgs.msg import SmachContainerStatus
from std_msgs.msg import Bool, UInt32
from hpp.corbaserver import Client as HppClient

cameraInfoString='''<?xml version="1.0"?>
<root>
  <!--This file stores intrinsic camera parameters used
   in the vpCameraParameters Class of ViSP available
   at https://visp.inria.fr/download/ .
   It is constructed by reading the /camera_info topic.
   WARNING: distortion coefficients are not reported. -->
  <camera>
    <!--Name of the camera-->
    <name>Camera</name>
    <!--Size of the image on which camera calibration was performed-->
    <image_width>{width}</image_width>
    <image_height>{height}</image_height>
    <!--Intrinsic camera parameters computed for each projection model-->
    <model>
      <!--Projection model type-->
      <type>perspectiveProjWithoutDistortion</type>
      <!--Pixel ratio-->
      <px>{px}</px>
      <py>{py}</py>
      <!--Principal point-->
      <u0>{u0}</u0>
      <v0>{v0}</v0>
    </model>
    <model>
      <!--Projection model type-->
      <type>perspectiveProjWithDistortion</type>
      <!--Pixel ratio-->
      <px>{px}</px>
      <py>{py}</py>
      <!--Principal point-->
      <u0>{u0}</u0>
      <v0>{v0}</v0>
      <!--Distorsion-->
      <kud>0</kud>
      <kdu>0</kdu>
    </model>
  </camera>
</root>
'''
# Write an image as read from a ROS topic to a file in png format
def writeImage(image, filename):
    count = 0
    a = np.zeros(3*image.width*image.height, dtype=np.uint8)
    a = a.reshape(image.height, image.width, 3)
    for y in range(image.height):
        for x in range(image.width):
            for c in range(3):
                a[y,x,c] = image.data[count]
                count +=1
    cv.imwrite(filename, a)

## Control calibration motions on UR10 robot
#
#  iteratively
#    - trigger motions planned by HPP by publishing in "/agimus/start_path"
#      topic,
#    - wait for end of motion by listening "/agimus/status/running" topic,
#    - record tf transformation between world and camera frames
#    - record image
class CalibrationControl (object):
    endEffectorFrame = "ref_camera_link"
    origin = "world"
    maxDelay = rospy.Duration (1,0)
    joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    def __init__ (self) :
        rospy.init_node ('calibration_control')
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration (1,0))
        self.tf2Listener = tf2_ros.TransformListener(self.tfBuffer)
        self.pubStartPath = rospy.Publisher ("/agimus/start_path", UInt32,
                                             queue_size=1)
        self.subRunning = rospy.Subscriber ("/agimus/status/running", Bool,
                                            self.runningCallback)
        self.subRosJointState = rospy.Subscriber ("/joint_states", JointState,
                                                  self.rosJointStateCallback)
        self.subStatus = rospy.Subscriber \
                         ("/agimus/agimus/smach/container_status",
                          SmachContainerStatus, self.statusCallback)
        self.subImage = rospy.Subscriber ("/camera/color/image_raw", Image,
                                                   self.imageCallback)
        self.subCameraInfo = rospy.Subscriber("/camera/color/camera_info",
                               CameraInfo, self.cameraInfoCallback)
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
        if not self.errorOccured:
            # wait to be sure that the robot is static
            rospy.sleep(10.)
            print("Collect data.")
            self.collectData ()

    def collectData (self):
        measurement = dict ()
        # record position of camera
        try:
            self.wMc = self.tfBuffer.lookup_transform\
                (self.origin, self.endEffectorFrame, rospy.Time(0))
            measurement["wMc"] = self.wMc
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.wMc = None
            rospy.loginfo ("No camera pose in tf")
        now = rospy.Time.now ()
        # Get joint values.
        if self.rosJointStates:
            measurement ["joint_states"] = self.rosJointStates
        # Get image.
        t = self.image.header.stamp
        # Check that data is recent enough
        if abs (now - t) < self.maxDelay:
            measurement["image"] = self.image
        else:
            rospy.loginfo ("time latest image from now: {}".
                           format ((now - t).secs + 1e-9*(now - t).nsecs))
        self.measurements.append (measurement)

    def save (self, directory):
        # write camera.xml from camera_info topic
        px = self.cameraInfo.K[0 * 3 + 0];
        py = self.cameraInfo.K[1 * 3 + 1];
        u0 = self.cameraInfo.K[0 * 3 + 2];
        v0 = self.cameraInfo.K[1 * 3 + 2];
        width = self.cameraInfo.width
        height= self.cameraInfo.height
        with open(directory + '/camera.xml', 'w') as f:
            f.write(cameraInfoString.format(width=width, height=height,
                                            px=px, py=py, u0=u0, v0=v0))
        # write urdf description of robot
        robotString = rospy.get_param('/robot_description')
        with open(directory + '/robot.urdf', 'w') as f:
            f.write(robotString)
        count=0
        for measurement in self.measurements:
            if not "image" in measurement.keys() or \
               not "wMc" in measurement.keys():
                continue
            count+=1
            writeImage(measurement['image'], directory + "/image-{}.png".\
                       format(count))
            with open(directory + "/pose_fPe_{}.yaml".format(count), 'w') as f:
                tf = measurement['wMc']
                rot = eigenpy.Quaternion(tf.transform.rotation.w,
                                         tf.transform.rotation.x,
                                         tf.transform.rotation.y,
                                         tf.transform.rotation.z)
                aa = eigenpy.AngleAxis(rot)
                utheta = aa.angle*aa.axis
                f.write("rows: 6\n")
                f.write("cols: 1\n")
                f.write("data:\n")
                f.write("- [{}]\n".format(tf.transform.translation.x))
                f.write("- [{}]\n".format(tf.transform.translation.y))
                f.write("- [{}]\n".format(tf.transform.translation.z))
                f.write("- [{}]\n".format(utheta[0]))
                f.write("- [{}]\n".format(utheta[1]))
                f.write("- [{}]\n".format(utheta[2]))
            if "joint_states" in measurement:
                with open(directory + f"/configuration_{count}", 'w') as f:
                    line = ""
                    q = len(measurement ["joint_states"])* [None]
                    for jn, jv in zip(self.jointNames,
                                      measurement ["joint_states"]):
                        q[self.joints.index(jn)] = jv
                    for jv in q:
                        line += f"{jv},"
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

    def imageCallback(self, msg):
        self.image = msg

    def cameraInfoCallback(self, msg):
        self.cameraInfo = msg
        self.subCameraInfo = None

    def sotJointStateCallback (self, msg):
        self.sotJointStates = msg.data

    def rosJointStateCallback (self, msg):
        self.rosJointStates = msg.position
        if not self.jointNames:
            self.jointNames = msg.name

def playAllPaths (startIndex):
    i = startIndex
    while i < nbPaths:
        cc.playPath (i)
        if not cc.errorOccured:
            print("Ran {}".format(i))
            i+=1
        #rospy.sleep (1)

if __name__ == '__main__':
    cc = CalibrationControl ()
    nbPaths = cc.hppClient.problem.numberPaths ()
    #playAllPaths (0)
