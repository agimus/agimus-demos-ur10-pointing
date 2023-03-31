from PyQt5.QtWidgets import QWidget, QGroupBox, QVBoxLayout, QHBoxLayout,\
 QPushButton, QLabel, QLineEdit, QComboBox, QDoubleSpinBox,\
  QSlider, QListView, QAbstractItemView, QCheckBox, QAbstractItemView, QDialog
from PyQt5.QtCore import QStringListModel, Qt
import math


class Teach_tab(QWidget):
    controlled_frames = ['ur10e_d435_mount_link', 'camera_color_optical_frame','ur10e_tooltip_link']
    deg = False

    # constructor
    def __init__(self,parent, ri, pg,robot, qApplication):
        super().__init__()
        self.ri = ri
        self.pg = pg
        self.robot = robot
        self.MainWindow = parent
        self.qApplication = qApplication

        self.tab1_layout = QVBoxLayout(self)

        # Add a combobox to tab1 and a label in the same row
        self.row = QHBoxLayout(self)
        self.row.addWidget(QLabel('Control TCP'))
        # add the combobox
        self.combo = QComboBox(self)
        self.combo.addItems(self.controlled_frames)
        # call the function when the combobox is changed
        self.combo.currentIndexChanged.connect(self.update_values)
        self.row.addWidget(self.combo)
        self.tab1_layout.addLayout(self.row)

        # Cartesian Group
        cartesian_group = QGroupBox('Cartesian')
        cartesian_layout = QVBoxLayout()

        #get the joint names (remove the last one as it is the part)
        self.jointNames = [joint.replace('ur10e/', '') for joint in self.robot.getJointNames()[:-1]]
        
        # dictionary of joint names and their values( default at 0)
        self.jointValues = dict(zip(self.jointNames, [0]*len(self.jointNames)))

        self.cartesian_spinBoxReal = []
        self.cartesian_checkBox = []

        for label in ['x', 'y', 'z','roll', 'pitch','yaw']:
            row = QHBoxLayout(self)
            # Checbox to constrain the movement on the axis, default is checked
            checkBox = QCheckBox(label+" : "); row.addWidget(checkBox)
            checkBox.setChecked(True)
            self.cartesian_checkBox.append(checkBox)

            spinBoxReal = QDoubleSpinBox(decimals=3, singleStep=0.05 , value=0 ); row.addWidget(spinBoxReal)
            # set no maximum and no minimum
            spinBoxReal.setMaximum(99)
            spinBoxReal.setMinimum(-99)
            self.cartesian_spinBoxReal.append(spinBoxReal)
            
            incLabal = QLabel('Increment'); row.addWidget(incLabal)
            incrementSpinBox = QDoubleSpinBox(decimals=2, singleStep=0.01, value=0.05); row.addWidget(incrementSpinBox)

            incrementSpinBox.valueChanged.connect(lambda value, spinBoxReal=spinBoxReal: spinBoxReal.setSingleStep(value))
            cartesian_layout.addLayout(row)
        
        cartesian_group.setLayout(cartesian_layout)
        self.tab1_layout.addWidget(cartesian_group)

        
        # Add a button to plan and execute the path with a steps checkbox in the same row
        self.row = QHBoxLayout(self)
        self.plan_execute_button = QPushButton('Plan and Execute')
        self.plan_execute_button.clicked.connect(self.plan_execute_cartesian)
        self.row.addWidget(self.plan_execute_button)
        self.steps_checkbox = QCheckBox('Steps')
        self.row.addWidget(self.steps_checkbox)
        # Add a copy button
        self.copy_button = QPushButton('Copy')
        self.copy_button.clicked.connect(self.copy_cartesien_to_clipboard)
        self.row.addWidget(self.copy_button)

        # Add reset button
        self.reset_button = QPushButton('Reset')
        self.reset_button.clicked.connect(self.update_values)
        self.row.addWidget(self.reset_button)

        cartesian_layout.addLayout(self.row)

        # Create a new group " Joints "
        joints_group = QGroupBox('Joints')
        joints_layout = QVBoxLayout()

        self.sliders = []
        # Add 6 sliders for the joints
        for label in self.jointNames:
            lowerBound = round( self.robot.getJointBounds(f"ur10e/{label}")[0], 2)
            upperBound = round( self.robot.getJointBounds(f"ur10e/{label}")[1], 2)
            # row for the label
            rowLabel = QHBoxLayout(self)
            uiLabel = QLabel(label + ' :'); rowLabel.addWidget(uiLabel)
            # Add a spinBox for the joint value and a label "->" and a spinbox for the result increment and a label "increment" in th rowLabel
            spinBoxReal = QDoubleSpinBox(decimals=2, singleStep=0.01, value=0); rowLabel.addWidget(spinBoxReal)
            # arrowLabel = QLabel('->'); rowLabel.addWidget(arrowLabel)
            # differenceSpinBox = QDoubleSpinBox(decimals=2, singleStep=0.05, value=0); rowLabel.addWidget(differenceSpinBox)
            # incrementLabel = QLabel('Increment'); rowLabel.addWidget(incrementLabel)
            # set the minimum and maximum of the spinboxReal and the differenceSpinBox
            spinBoxReal.setMinimum(lowerBound*100)
            spinBoxReal.setMaximum(upperBound*100)
            # differenceSpinBox.setMinimum(lowerBound*100)
            # differenceSpinBox.setMaximum(upperBound*100)
            # Add the rowLabel to the joints_layout
            joints_layout.addLayout(rowLabel)

            row = QHBoxLayout(self)
            # Add a label for the lower bound
            lowerBoundLabel = QLabel(str( lowerBound))
            row.addWidget(lowerBoundLabel)
            slider = QSlider(Qt.Horizontal); row.addWidget(slider)
            # Add a label for the upper bound
            upperBoundLabel = QLabel(str( upperBound))
            row.addWidget(upperBoundLabel)
            self.sliders.append(slider)
            slider.setMinimum( lowerBound*100)
            slider.setMaximum( upperBound*100)
            slider.setValue(0)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(15.5)
            joints_layout.addLayout(row)

            # Connect the spinboxReal to the slider
            spinBoxReal.valueChanged.connect(lambda value, slider=slider: slider.setValue(value*100))
            # Connect the slider to the spinboxReal
            slider.valueChanged.connect(lambda value, spinBoxReal=spinBoxReal: spinBoxReal.setValue(value/100))

        
        # Create a row
        row = QHBoxLayout(self)

        # Add a button to plan and execute the movement
        planButton = QPushButton('Plan and execute')
        planButton.clicked.connect(self.planAndExecute)
        row.addWidget(planButton)
        #  a checkbox to enable/disable the steps
        self.enableSteps = QCheckBox('Steps')
        row.addWidget(self.enableSteps)
        # Add a reset button to reset
        resetButton = QPushButton('Reset')
        resetButton.clicked.connect(self.update_values)
        row.addWidget(resetButton)
        # Add a button to copy the joint values to the clipboard
        copyButton = QPushButton('Copy')
        copyButton.clicked.connect(self.copy_joints_to_clipboard)
        row.addWidget(copyButton)

        # Add the row to the joints_layout
        joints_layout.addLayout(row)

        joints_group.setLayout(joints_layout)
        self.tab1_layout.addWidget(joints_group)

        # Add a group "configuration"
        config_group = QGroupBox('Configuration')
        config_layout = QVBoxLayout()
        # Add a input box for the name of the configuration
        config_layout.addWidget(QLabel('Name'))
        self.config_name = QLineEdit()
        config_layout.addWidget(self.config_name)
        # Add a button save to config
        config_layout.addWidget(QPushButton('Save to config', clicked=self.save_to_config))

        config_group.setLayout(config_layout)
        self.tab1_layout.addWidget(config_group)

        # Create another group "Plan to configuration"
        plan_group = QGroupBox('Plan to configuration')
        plan_layout = QVBoxLayout()
        # Add a list view to show the configurations
        self.config_list = QListView()
        # only allow one selection
        self.config_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.config_list.setModel(QStringListModel(self.pg.configs.keys())) 
        plan_layout.addWidget(self.config_list)
        # Add a button to plan to the selected configuration and a checkbox called "Steps" in the same row
        row = QHBoxLayout()
        row.addWidget(QPushButton('Plan to config', clicked=self.plan_to_config))
        self.checkbox_steps = QCheckBox('Steps')
        row.addWidget(self.checkbox_steps)
        # Add a copy to clipboard button
        copy_button = QPushButton('Copy to clipboard')
        copy_button.clicked.connect(self.copy_config_to_clipboard)
        row.addWidget(copy_button)

        plan_layout.addLayout(row)

        # plan_layout.addWidget(QPushButton('Plan to config', clicked=self.plan_to_config))
        plan_group.setLayout(plan_layout)
        self.tab1_layout.addWidget(plan_group)

        self.update_values()


    def save_to_config(self):
        # get the name of the configuration
        name = self.config_name.text()
        # get the current joint position
        q0 = self.robot.getCurrentConfig()
        q = self.ri.getCurrentConfig(q0)
        self.pg.setConfig(str(name), q)
        self.update_config_list()
        # show a status message
        self.MainWindow.statusBar().showMessage('Saved configuration ' + name)


    def copy_config_to_clipboard(self):
        # get the name of the selected configuration
        selected = self.config_list.selectedIndexes()[0].data()
        # get the configuration from self.pg.configs
        config = self.pg.configs[selected]
        # convert the configuration to a string
        config = str(config)
        # copy the string to the clipboard
        clipboard = self.qApplication.clipboard()
        clipboard.setText(f"{selected} : {config}")

        # Show a message for 2 seconds
        self.MainWindow.statusBar().showMessage(f'Configuration "{selected}" copied to clipboard', 2000)

    def copy_cartesien_to_clipboard(self):
        frame = self.combo.currentText()
        tcp = self.ri.getFrame(frame, euler=True)
        clipboard = self.qApplication.clipboard()
        clipboard.setText(f"{frame} : {tcp}")

        # Show a message for 2 seconds
        self.MainWindow.statusBar().showMessage(f'Cartesian "{frame}" copied to clipboard', 2000)

    def copy_joints_to_clipboard(self):
        q0 = self.robot.getCurrentConfig()
        joints = self.ri.getCurrentConfig(q0)
        clipboard = self.qApplication.clipboard()
        clipboard.setText(f"{joints}")

        # Show a message for 2 seconds
        self.MainWindow.statusBar().showMessage(f'Joints copied to clipboard', 2000)

    def update_values(self):
        # get combo box value
        tcp = self.ri.getFrame(self.combo.currentText(), euler=True)
        # round the values to 3 decimals
        tcp = [round(i, 3) for i in tcp]
        # update the spin boxes real
        for i in range(6):
            self.cartesian_spinBoxReal[i].setValue(round(tcp[i], 3))

        q0 = self.robot.getCurrentConfig()
        joints = self.ri.getCurrentConfig(q0)
        joints = [round(i, 3) for i in joints]
        for i in range(6):
            self.sliders[i].setValue(joints[i]*100)
            self.jointValues[self.jointNames[i]] = joints[i]
        

    def update_config_list(self):
        # update list view with the keys of self.pg.configs
        # get name of the selected configuration in the list view
        if len(self.config_list.selectedIndexes()) > 0:
            selected = self.config_list.selectedIndexes()[0].data()
            self.config_list.setModel(QStringListModel(self.pg.configs.keys()))
            # select the configuration again by it's name
            for i in range(self.config_list.model().rowCount()):
                if self.config_list.model().index(i, 0).data() == selected:
                    self.config_list.setCurrentIndex(self.config_list.model().index(i, 0)) 
        else:
            self.config_list.setModel(QStringListModel(self.pg.configs.keys()))     
    
    def planAndExecute(self):
        # get the value of joint slider and convert it to a list
        joints = [self.sliders[i].value()/100 for i in range(6)]
        # add partPose to the list
        q0 = self.robot.getCurrentConfig()
        q = self.ri.getCurrentConfig(q0)
        # replace the first 6 values of the list with the joint values
        q[:6] = joints
        # get steps from the checkbox
        steps = self.enableSteps.isChecked()
        
        # plan
        pid,_ = self.pg.planTo(q)
        print('plan id : ' + str(pid))
        if pid:
            if steps:
                self.show_dialog(pid, type='joint')
            else:
                self.pg.demo_execute(pid, steps=False)
        else:
            print('plan failed')
        # reset the joint sliders by the real self.robot joint position
        self.update_values()
        
    def plan_execute_cartesian(self):
        # Get the values of the spinbox in the cartesian_spinBoxReal list
        target = [self.cartesian_spinBoxReal[i].value() for i in range(len(self.cartesian_spinBoxReal))]
        if self.deg:
            # convert the last 3 values to radians
            target[-3:] = [math.radians(i) for i in target[-3:]]
        # Get the frame from combo box
        frame = self.combo.currentText()
        # get mask from the checkboxes
        mask = [self.cartesian_checkBox[i].isChecked() for i in range(len(self.cartesian_checkBox))]
        # get the checked state of the steps checkbox
        steps = self.steps_checkbox.isChecked()

        # plan
        pid = self.pg.planGoTarget(target, frame, mask)
        if pid:
            if steps:
                self.show_dialog(pid,type='cartesian')
            else:
                self.pg.demo_execute(pid, steps=False)
        else:
            print('plan failed')

        self.update_values()

    
    # function that show a dialog to reshow the path or execute it or cancel or calculate another path
    def show_dialog(self, pid, type):
        # create a dialog
        dialog = QDialog()
        # create a layout
        layout = QVBoxLayout()
        # create a label
        label = QLabel('Plan id : ' + str(pid))
        # create a button to show the path
        show_button = QPushButton('Show path')
        # create a button to execute the path
        execute_button = QPushButton('Execute path')
        # create a button to calculate another path
        calculate_button = QPushButton('Calculate another path')
        # create a button to cancel
        cancel_button = QPushButton('Cancel')
        # add the buttons to the layout
        layout.addWidget(label)
        layout.addWidget(show_button)
        layout.addWidget(execute_button)
        layout.addWidget(calculate_button)
        layout.addWidget(cancel_button)
        # add the layout to the dialog
        dialog.setLayout(layout)
        # connect the buttons to the functions
        show_button.clicked.connect(lambda: self.pg.pp(pid))
        # execute the plan and close the dialog
        execute_button.clicked.connect(lambda: self.pg.demo_execute(pid, steps=False))
        execute_button.clicked.connect(dialog.close)
        if type == 'joint':
            calculate_button.clicked.connect(lambda: self.planAndExecute())
        elif type == 'cartesian':
            calculate_button.clicked.connect(lambda: self.plan_execute_cartesian())
        elif type == 'config':
            calculate_button.clicked.connect(lambda: self.plan_to_config())
        calculate_button.clicked.connect(dialog.close)
        cancel_button.clicked.connect(dialog.close)
        # show the dialog
        dialog.exec_()

    def plan_to_config(self):
        # get the checked state of the checkbox
        steps = self.checkbox_steps.isChecked()
        # get the selected configuration
        selected = self.config_list.selectedIndexes()
        if len(selected) == 0:
            return
        # get the name of the configuration
        name = selected[0].data()
        # get the configuration
        pid,_ = self.pg.planTo(name)
        if pid:
            if steps:
                self.show_dialog(pid, type='config')
            else:
                self.pg.demo_execute(pid, steps=False)
        self.update_values()

        
    def update_joints(self, joints_list):   
        for label in self.joint_pos_labels:
            label.setText(str(joints_list[self.joint_pos_labels.index(label)]))
