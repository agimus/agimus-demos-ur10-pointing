from PyQt5.QtWidgets import QMainWindow, QTabWidget

from TeachWidget.tabs.calibration import Calibration_tab
from TeachWidget.tabs.parameters import Parameters_tab
from TeachWidget.tabs.teach import Teach_tab

class MainWindow(QMainWindow):

    def __init__(self,ri, pg, robot, ps, qApplication):
        super().__init__()
        self.ri = ri
        self.pg = pg
        self.robot = robot
        self.ps = ps
        self.qApplication = qApplication

        # Create 3 tabs : teach, parameters, calibration
        self.tabs = QTabWidget()
        self.tabs.resize(400, 300)
        self.tab1 = Teach_tab(self, self.ri, self.pg, self.robot, self.qApplication)
        self.tab2 = Parameters_tab(self, self.ri, self.pg, self.robot, self.ps)
        self.tab3 = Calibration_tab(self, self.ri, self.pg, self.robot)

        # Add tabs
        self.tabs.addTab(self.tab1, "Teach")
        self.tabs.addTab(self.tab2, "Parameters")
        self.tabs.addTab(self.tab3, "Calibration")
        self.setCentralWidget(self.tabs)
