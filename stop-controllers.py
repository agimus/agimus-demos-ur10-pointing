#!/usr/bin/env python
from __future__ import print_function
import rospy
from controller_manager_msgs.srv import SwitchController

rospy.init_node('StoppingControllers')
swCtl = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
for ctrl in [ 'scaled_pos_joint_traj_controller']:
    resp = swCtl(stop_controllers=[ ctrl, ], strictness=2)
    if resp.ok:
        print("Stopped", ctrl)
    else:
        print("FAILED to stop ", ctrl)
