from PyQt5.QtWidgets import QWidget, QGroupBox, QVBoxLayout, QPushButton, QLabel, QSlider
from PyQt5.QtCore import Qt

class Parameters_tab(QWidget):

    # constructor
    def __init__(self,parent, ri, pg,robot, ps):
        super().__init__()
        self.ri = ri
        self.pg = pg
        self.robot = robot
        self.MainWindow = parent
        self.ps = ps

        self.tab2_layout = QVBoxLayout(self)
        # Add a Speed and Acceleration profile group to tab 2 
        speed_group = QGroupBox('Speed and Acceleration profile')
        speed_layout = QVBoxLayout()
        # Add a slider for the speed
        speed_layout.addWidget(QLabel('Speed'))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setValue(95)
        self.speed_slider.setTickInterval(1)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        speed_layout.addWidget(self.speed_slider)
        # Add a slider for the acceleration
        speed_layout.addWidget(QLabel('Acceleration'))
        self.accel_slider = QSlider(Qt.Horizontal)
        self.accel_slider.setMinimum(0)
        self.accel_slider.setMaximum(100)
        self.accel_slider.setValue(50)
        self.accel_slider.setTickInterval(1)
        self.accel_slider.setTickPosition(QSlider.TicksBelow)
        speed_layout.addWidget(self.accel_slider)
        # Add a button to set the speed and acceleration
        speed_layout.addWidget(QPushButton('Set speed and acceleration', clicked=self.set_speed_accel))

        # reduce the size of the group
        speed_group.setMaximumHeight(200)

        speed_group.setLayout(speed_layout)
        self.tab2_layout.addWidget(speed_group)
        # align top
        self.tab2_layout.setAlignment(Qt.AlignTop)

        self.setLayout(self.tab2_layout)


    def set_speed_accel(self):
        # get the value of the speed slider
        speed = self.speed_slider.value()
        # get the value of the acceleration slider
        accel = self.accel_slider.value()
        print('speed : ' + str(speed))
        print('accel : ' + str(accel))
        # set the speed and acceleration
        self.ps.setParameter('SimpleTimeParameterization/safety', speed/100)
        self.ps.setParameter('SimpleTimeParameterization/maxAcceleration', accel/100)
        # status message 
        self.MainWindow.statusBar().showMessage('Speed : ' + str(speed) + '% -  Acceleration : ' + str(accel) + '% set')

       