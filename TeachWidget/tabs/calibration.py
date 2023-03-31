from PyQt5.QtWidgets import QWidget, QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, QFileDialog
import os
from play_path import CalibrationControl
from PyQt5.QtCore import Qt


class Calibration_tab(QWidget):

    def __init__(self,parent, ri, pg,robot):
        super().__init__()
        self.init_calibration()
        self.ri = ri
        self.pg = pg
        self.robot = robot
        self.MainWindow = parent

        # Add a third tab "calibration"
        self.tab3_layout = QVBoxLayout()
        self.setLayout(self.tab3_layout)
        
        # Add a group "calibration"
        calib_group = QGroupBox('Calibration')
        calib_layout = QVBoxLayout()
        # Add a button to take a measurement
        calib_layout.addWidget(QPushButton('Take measurement', clicked=self.take_measurement))
        # Add a label that shows the number of measurements taken in a row
        row = QHBoxLayout()
        row.addWidget(QLabel('Measurements taken:'))
        self.measurement_count = QLabel('0')
        row.addWidget(self.measurement_count)
        calib_layout.addLayout(row)

        # Add a row with an input fro selecting a folder and a button to save the measurements
        row = QHBoxLayout()
        self.calib_folder = QLineEdit()
        self.calib_folder.setText(os.getcwd())
        row.addWidget(self.calib_folder)
        row.addWidget(QPushButton('Select folder', clicked=self.select_folder))
        calib_layout.addLayout(row)
        calib_layout.addWidget(QPushButton('Save measurements', clicked=self.save_measurements))

        calib_group.setLayout(calib_layout)
        self.tab3_layout.addWidget(calib_group)

        # align top
        self.tab3_layout.setAlignment(Qt.AlignTop)


    def init_calibration(self):
        self.cc = CalibrationControl ()

    def take_measurement(self):
        self.cc.collectData()
        self.measurement_count.setText(str(len(self.cc.measurements)))

    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, 'Select folder')
        self.calib_folder.setText(folder)

    def save_measurements(self):
        if self.calib_folder.text() != '':
            self.cc.save(self.calib_folder.text())
            # reset the measurement count
            self.measurement_count.setText('0')
            # Reset the calibration control measurements
            self.cc.measurements = []
            self.MainWindow.statusBar().showMessage('Saved measurements to ' + self.calib_folder.text())
        else:
            self.MainWindow.statusBar().showMessage('Please select a folder first')