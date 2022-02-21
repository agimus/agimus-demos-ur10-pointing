from csv import reader
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QWidget,
)
from qt_gui.plugin import Plugin
import QtGui

import rospy
import tf
import numpy as np
from dynamic_graph_bridge_msgs.srv import RunCommand
from agimus_sot.srdf_parser import parse_srdf
from pinocchio import integrate, JointModelFreeFlyer, Model, Quaternion, \
    SE3

def tfToSE3(trans, rot):
    p = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])
    t = np.array(trans).reshape(3,1)
    return SE3(p,t)

def cross(v):
    return np.cross(v, np.identity(v.shape[0]) * -1)

class SE3Integrator (object):
    """
    Integrate a velocity on SE3
    """
    def __init__(self):
        self.model = Model()
        self.model.addJoint(0, JointModelFreeFlyer(), SE3(), "SE3")

    def __call__(self, T, nu):
        """
        Integrate se3 velocity from SE3 element T
          - T: instance of SE3
          - nu: numpy array of dimension 6 (v,omega)
        """
        q0 = np.array(7*[0.])
        q0[0:3] = T.translation
        q0[3:7] = Quaternion(T.rotation).coeffs()
        q = integrate(self.model,q0,nu)
        return SE3(Quaternion(x=q[3], y=q[4], z=q[5], w=q[6]), q[0:3])

##
#  Qt Widget to perform hand eye calibration of the UR10 robot
#
# This class provides methods to
#
#   - align the tooltip with a hole via an offset of the gripper
#     position in the end effector frame,
#   - recording the offset together with the part pose in the camera frame,
#   - compute the camera pose in the end effector frame.

class TooltipCalibration(Plugin):
    _verbose = False
    runCommandService="/run_command"
    initCommands = [
        "import numpy as np",
        "from dynamic_graph.sot.core.feature_pose import FeaturePose",
        "from pinocchio import SE3",
        "from dynamic_graph import get_entity_list",
        "tc_entities = get_entity_list()",
        "tc_f = FeaturePose('pregrasp___ur10e/gripper___part/handle_0_feature')",
        "print(tc_entities)"]
    outputFile = None
    cameraFrame = 'ref_camera_link'
    objectFrame = 'part/base_link'
    objectFrameMeas = 'part/base_link_measured'
    endEffectorFrame = 'ur10e_d435_mount_link'
    tooltipFrame = 'ur10e_tooltip_link'
    objectName = 'part'

    def __init__(self, context):
        super(TooltipCalibration, self).__init__(context)
        self.setObjectName("Tooltip calibration")
        self.measurements = list()
        # Create QWidget
        self._widget = QWidget()
        layout = QGridLayout()
        self._widget.setLayout(layout)
        self._translation = [0,0,0]
        row = 0
        label = QLabel("gripper:")
        gripperId = QSpinBox()
        gripperId.setRange(0, 0)
        gripperId.valueChanged.connect(self.gripperIdChanged)
        self.gripperName = QLabel("")
        layout.addWidget(label, row, 0)
        layout.addWidget(self.gripperName, row, 1)
        layout.addWidget(gripperId, row, 2)

        row += 1
        label = QLabel("handle:")
        handleId = QSpinBox()
        handleId.setRange(0, 0)
        handleId.valueChanged.connect(self.handleIdChanged)
        self.handleName = QLabel("")
        layout.addWidget(label, row, 0)
        layout.addWidget(self.handleName, row, 1)
        layout.addWidget(handleId, row, 2)

        row += 1
        xSlider = QSlider(Qt.Horizontal)
        xSlider.setMinimum(-100)
        xSlider.setMaximum(100)
        xSlider.setValue(0)
        xSlider.valueChanged.connect(self.xChanged)
        label = QLabel("x")
        #label.setFont(QtGui.QFont("Timesroman", 12))
        self.xValue = QLabel("0.000")

        layout.addWidget(label, row, 0)
        layout.addWidget(xSlider, row, 1)
        layout.addWidget(self.xValue, row, 2)

        row+=1

        ySlider = QSlider(Qt.Horizontal)
        ySlider.setMinimum(-100)
        ySlider.setMaximum(100)
        ySlider.setValue(0)
        ySlider.valueChanged.connect(self.yChanged)
        label = QLabel("y")
        #label.setFont(QtGui.QFont("Timesroman", 12))
        self.yValue = QLabel("0.000")

        layout.addWidget(label, row, 0)
        layout.addWidget(ySlider, row, 1)
        layout.addWidget(self.yValue, row, 2)

        row+=1
        zSlider = QSlider(Qt.Horizontal)
        zSlider.setMinimum(-100)
        zSlider.setMaximum(100)
        zSlider.setValue(0)
        zSlider.valueChanged.connect(self.zChanged)
        label = QLabel("z")
        #label.setFont(QtGui.QFont("Timesroman", 12))
        self.zValue = QLabel("0.000")

        layout.addWidget(label, row, 0)
        layout.addWidget(zSlider, row, 1)
        layout.addWidget(self.zValue, row, 2)

        row+=1
        openFileButton = QPushButton('open file')
        openFileButton.clicked.connect(self.openFileRequested)
        layout.addWidget(openFileButton, row, 0)

        saveButton = QPushButton('save measurement')
        saveButton.clicked.connect(self.saveRequested)
        layout.addWidget(saveButton, row, 1)

        closeFileButton = QPushButton('close file')
        closeFileButton.clicked.connect(self.closeFileRequested)
        layout.addWidget(closeFileButton, row, 2)

        row+=1

        solveButton = QPushButton('solve')
        solveButton.clicked.connect(self.solve)
        layout.addWidget(solveButton, row, 1)

        row+=1
        # Add widget to the user interface
        context.add_widget(self._widget)
        # Parse srdf files to get available grippers and handles
        self.parseSrdf()
        handleId.setRange(0, len(self.handles)-1)
        if len(self.handles)>0:
            self.handle = self.handles[0]
            self.handleName.setText(self.handle)
        gripperId.setRange(0, len(self.grippers)-1)
        if len(self.grippers)>0:
            self.gripperName.setText(self.grippers[0])
        # Wait for service dynamic_graph_bridge/run_command
        rospy.wait_for_service(self.runCommandService)
        self._runCommand = rospy.ServiceProxy(self.runCommandService,
                                              RunCommand)
        self.initializeSot()

        self._tfListener = tf.TransformListener()
        readFrame = False
        while not readFrame:
            try:
                # Get initial pose of end-effector in camera frame.
                (trans,rot) = self._tfListener.lookupTransform\
                    (self.cameraFrame, self.endEffectorFrame, rospy.Time(0))
                self.cMe = tfToSE3(trans, rot)
                # Get nominal position of tooltip in end-effector frame
                (trans,rot) = self._tfListener.lookupTransform\
                    (self.endEffectorFrame, self.tooltipFrame, rospy.Time(0))
                self.et = np.array(trans)
                readFrame = True
                self.computeHolePosition()
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                print('Failed to read information from tf, keep trying')
                rospy.sleep(1.)


    def xChanged(self, val):
        self.xValue.setText("{:.3f}".format(1e-3*val))
        self._translation[0] = 1e-3*val
        self.setSignalValue()

    def yChanged(self, val):
        self.yValue.setText("{:.3f}".format(1e-3*val))
        self._translation[1] = 1e-3*val
        self.setSignalValue()

    def zChanged(self, val):
        self.zValue.setText("{:.3f}".format(1e-3*val))
        self._translation[2] = 1e-3*val
        self.setSignalValue()

    def handleIdChanged(self, val):
        self.handle = self.handles[val]
        self.handleName.setText(self.handle)
        self.updateFeatureName()
        self.runCommand("tc_f = FeaturePose('{}')".format(self._featureName))
        self.computeHolePosition()

    def gripperIdChanged(self, val):
        self.gripper = self.grippers[val]
        self.gripperName.setText(self.gripper)
        self.updateFeatureName()
        self.runCommand("tc_f = FeaturePose('{}')".format(self._featureName))

    def updateFeatureName(self):
        self._featureName = 'pregrasp___{}___{}_feature'.format\
            (self.gripper, self.handle)
        if not self._featureName in self.entities:
            rospy.logerr("{} is not an existing entity".format\
                         (self._featureName))

    def openFileRequested(self):
        filename, _ = QFileDialog.getSaveFileName()
        if filename != '':
            self.outputFile = open(filename, 'w')

    def saveRequested(self, x):
        if self.outputFile is None:
            raise RuntimeError('Open a file first.')
        dx, dy, dz = self._translation
        # Read tf
        (trans,rot) = self._tfListener.lookupTransform(self.cameraFrame,
                                                       self.objectFrameMeas,
                                                       rospy.Time(0))
        self.outputFile.write(f'offsets,{dx},{dy},{dz},')
        self.outputFile.write(f'cMo,{trans[0]},{trans[1]},{trans[2]},{rot[0]},{rot[1]},{rot[2]},{rot[3]},\n')

    def closeFileRequested(self):
        if self.outputFile:
            self.outputFile.close()

    ##
    #  Communication with the Stack of Tasks
    def _isNotError (self, runCommandAnswer):
        if len(runCommandAnswer.standarderror) != 0:
            return False, runCommandAnswer.standarderror
        return True, ""

    def runCommand (self, cmd):
        if self._verbose:
            rospy.loginfo(">> " + cmd)
        answer = self._runCommand (cmd)
        if self._verbose and len(answer.standardoutput) > 0:
            rospy.loginfo(answer.standardoutput)
        if len(answer.standardoutput) > 0:
            self._verbose and rospy.loginfo (answer.standardoutput)
        if len(answer.standarderror) > 0:
            rospy.logerr (answer.standarderror)
        return answer

    def initializeSot(self):
        for c in self.initCommands:
            answer = self.runCommand(c)
            success, msg = self._isNotError(answer)
            if not success:
                return
        self.entities = eval(answer.standardoutput)

    def setSignalValue(self):
        self.runCommand("tc_s = tc_f.signal('jaMfa')")
        self.runCommand("tc_M = tc_s.value")
        self.runCommand("tc_M[0,3] = {}".format(self._translation[0]))
        self.runCommand("tc_M[1,3] = {}".format(self._translation[1]))
        self.runCommand("tc_M[2,3] = {}".format(self._translation[2]))
        self.runCommand("tc_s.value = tc_M")

    def parseSrdf(self):
        if not rospy.has_param("/demo"):
            rospy.logerr("No ROS parameter /demo")
            return
        srdf = {}
        # retrieve objects from ros param
        demoDict = rospy.get_param("/demo")
        robotDict = demoDict["robots"]
        if len(robotDict) != 1:
            raise RuntimeError("One and only one robot is supported for now.")
        objectDict = demoDict["objects"]
        objects = list(objectDict.keys())
        # parse robot and object srdf files
        self._srdfDict = dict()
        for r, data in robotDict.items():
            self._srdfDict[r] = parse_srdf(srdf = data["srdf"]["file"],
                                     packageName = data["srdf"]["package"],
                                     prefix=r)
        for o, data in objectDict.items():
            self._srdfDict[o] = parse_srdf(srdf = data["srdf"]["file"],
                                     packageName = data["srdf"]["package"],
                                     prefix=o)
        self.grippers = list(demoDict["grippers"])
        if len(self.grippers) > 0: self.gripper = self.grippers[0]
        self.handles = list()
        for o in objects:
            self.handles.extend(sorted(list(self._srdfDict[o]
                                            ["handles"].keys())))

    def computeHolePosition(self):
        # get name of link that holds the handle
        holeLink = self.objectName + '/' + \
            self._srdfDict[self.objectName]['handles'][self.handle]['link']
        # get position of hole in link
        p = self._srdfDict[self.objectName]['handles'][self.handle]['position']
        lh = np.array(p[0:3])
        # get pose of link in part
        (trans, rot) = self._tfListener.lookupTransform(self.objectFrame,
                                                       holeLink, rospy.Time(0))
        oMl = tfToSE3(trans, rot)
        # Get position of hole in part frame
        self.oh = oMl.act(lh)

    def parseLine(self, line, i):
        # Check that line starts with joint_states
        if line[0] != 'offsets':
            raise RuntimeError('line {} does not start by keyword "offsets"'
                               .format(i))
        measurement = dict()
        try:
            measurement['offsets'] = np.array(list(map(float, line [1:4])))
        except ValueError as exc:
            raise SyntaxError(f'line {i+1}, tag "offsets": could not convert' +\
                              f' list {line [1:4]} to array')
        if line[4] != 'cMo':
            raise SyntaxError(f'line {i+1}, expected tag "cMo": got {line[4]}')
        try:
            v = list(map(float, line [5:12]))
            p = Quaternion(x=v[3], y=v[4], z=v[5], w=v[6])
            t = np.array(v[0:3]).reshape(3,1)
            measurement ['cMo'] = SE3(p,t)
        except ValueError as exc:
            raise SyntaxError(f'line {i+1}, tag "cMo": could not convert' +\
                              f' list {line [5:12]} to array')
        return measurement

    def readData(self, filename):
        with open(filename, 'r') as f:
            r = reader(f)
            for i, line in enumerate(r):
                self.measurements.append(self.parseLine(line, i))

    def solve(self, x):
        integration = SE3Integrator()
        # allocate vector and Jacobian matrix
        self.value = np.zeros(3*len(self.measurements))
        self.jacobian = np.zeros(6*3*len(self.measurements))
        self.jacobian.resize(3*len(self.measurements), 6)
        self.computeValueAndJacobian()
        for i in range(20):
            print(f'squared error = {sum(self.value*self.value)}')
            nu = -np.matmul(np.linalg.pinv(self.jacobian),self.value)
            self.cMe = integration(self.cMe, nu)

    def computeValueAndJacobian(self):
        for i, m in enumerate(self.measurements):
            self.value[i:i+3] = self.cMe.act(self.et +
                m['offsets'] - m['cMo'].act(self.oh)
            self.jacobian[i+0:i+3,0:3] = self.cMe.rotation
            self.jacobian[i+3:i+6,3:6] = -np.matmul(self.cMe.rotation,
                cross(self.et + m['offsets'])
