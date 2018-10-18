#!/bin/bash

USAGE="Usage: rosbag-calib rosbag_name"

if [ $# == 0 ] ; then
  echo $USAGE
  exit 1;
fi

rosbag record -l 1 /joint_states /rgbd/depth/camera_info /rgbd/depth/image_raw /rgbd/depth/points /rgbd/depth_registered/camera_info /rgbd/depth_registered/hw_registered/image_rect_raw /rgbd/depth_registered/image_raw /rgbd/depth_registered/points /rgbd/ir/camera_info /rgbd/ir/image /rgbd/rgb/camera_info /rgbd/rgb/high_res/camera_info /rgbd/rgb/high_res/image_raw /rgbd/rgb/image_raw /tf /checkerdetector/objectdetection_pose --bz2 -O $1

