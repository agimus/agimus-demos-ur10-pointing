# Copyright 2022 CNRS - Airbus SAS
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
