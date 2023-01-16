import numpy as np
from dynamic_graph import get_entity_list, factory_get_entity_class_list
from dynamic_graph.sot.core.operator import Inverse_of_matrixHomo, \
    Multiply_of_matrixHomo
from dynamic_graph.sot.core.feature_pose import FeaturePose
from agimus_sot.sot import ObjectLocalization
from pinocchio import SE3

OFFSET_X = 0.0070
OFFSET_Y = 0.0102
OFFSET_Z = -0.0086

R = np.matrix([[0,1,0],[0,0,1],[1,0,0]])
# x = up
# y = towards the left (when looking at the part from the gripper)
x = SE3(R, np.array([OFFSET_X,OFFSET_Y,OFFSET_Z]))

# ol = ObjectLocalization('part/base_link_measuredwrt_world_ol')
# ol.trigger(robot.device.control.time)

def getSignal(handle_id):
    f = FeaturePose('pregrasp___ur10e/gripper___part/handle_' + str(handle_id).zfill(2) + '_feature')
    s = f.signal("jaMfa")
    return s

def changejaMfa(handle_id, x):
    s = getSignal(handle_id)
    s.value = x.homogeneous

#changejaMfa(5,x)
for i in range(44):
    changejaMfa(i, x)

# supervisor.sots['Loop | 0-5'] = supervisor.sots['ur10e/gripper > part/handle_05 | f_12']