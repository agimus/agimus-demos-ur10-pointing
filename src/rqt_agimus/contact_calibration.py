from csv import reader
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QGridLayout,
    QPushButton,
    QWidget,
)
from qt_gui.plugin import Plugin
import QtGui

import rospy
import datetime
from std_msgs.msg import Float64

##
#  Qt Widget to trigger signal allowing robot to release contact
#

class ReleaseContact(Plugin):

    def __init__(self, context):
        super(ReleaseContact, self).__init__(context)
        self.setObjectName("Calibration by contact with planes")
        self.measurements = list()
        # Create QWidget
        self._widget = QWidget()
        layout = QGridLayout()
        self._widget.setLayout(layout)
        self._translation = [0,0,0]
        row = 0
        row+=1
        releaseContactButton = QPushButton('Release contact')
        releaseContactButton.clicked.connect(self.releaseContactRequested)
        layout.addWidget(releaseContactButton, row, 0)
        # Create publisher
        self.publisher = rospy.Publisher("/agimus/release_contact", Float64,
                                         queue_size = 1)
        self.start = datetime.datetime.now()
        # Add widget to the user interface
        context.add_widget(self._widget)

    def releaseContactRequested(self):
        self.publisher.publish((datetime.datetime.now() - self.start).
                               total_seconds())
