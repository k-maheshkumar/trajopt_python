import sys
from PyQt4 import QtGui, QtCore
import yaml
import numpy as np
from scripts.utils import yaml_paser as yaml
import itertools
class PlannerGui(QtGui.QWidget):
    def __init__(self):
        super(PlannerGui, self).__init__()

        file_path_prefix = '../../config/'
        sqp_config_file = file_path_prefix + 'sqp_config.yaml'

        sqp_yaml = yaml.ConfigParser(sqp_config_file)
        self.sqp_config = sqp_yaml.get_by_key("sqp")
        robot_config_file = file_path_prefix + 'robot_config.yaml'
        robot_yaml = yaml.ConfigParser(robot_config_file)
        self.robot_config = robot_yaml.get_by_key("robot")
        self.initUI()

    def initUI(self):
        hbox = QtGui.QHBoxLayout()
        sqp_grid = QtGui.QGridLayout()
        grid1 = QtGui.QGridLayout()

        # # Definition des Tracing Parameters widgets
        # trust_box_size_label = QtGui.QLabel("Trust region size")
        # max_trust_region_label = QtGui.QLabel("Max trust region size")
        # trust_box_size = QtGui.QSpinBox(self)
        # trust_box_size.setValue(self.sqp_config["trust_region_size"])
        # max_trust_region = QtGui.QSpinBox(self)
        # max_trust_region.setValue(self.sqp_config["max_trust_region_size"])
        # window = QtGui.QGroupBox(self)
        # window.setTitle("SQP solver Parameters")
        # hbox.addLayout(sqp_grid)
        # hbox.addLayout(grid1)


        sqp_labels = []
        sqp_spin_box = []
        sqp_grid = QtGui.QGridLayout()
        min_ = 0.00001
        max_ = 1
        # Definition des Tracing Parameters widgets
        for key, value in self.sqp_config.iteritems():
            sqp_labels.append({key :QtGui.QLabel(key)})
            sqp_spin_box.append({key: QtGui.QDoubleSpinBox(self)})

        sqp_spin_box = {k: v for d in sqp_spin_box for k, v in d.items()}
        sqp_labels = {k: v for d in sqp_labels for k, v in d.items()}

        sqp_param_count = 0
        hbox.addLayout(sqp_grid)
        for key, value in sqp_labels.iteritems():
            sqp_grid.addWidget(sqp_labels[key], sqp_param_count, 0)
            sqp_grid.addWidget(sqp_spin_box[key], sqp_param_count, 1)
            if self.sqp_config[key] is str:
                sqp_spin_box[key].setValue(float(self.sqp_config[key]))
            sqp_spin_box[key].setRange(min_, max_)
            self.connect(sqp_spin_box[key], QtCore.SIGNAL('valueChanged(double)'), self.change_value1)
            sqp_param_count += 1


        # trust_box_size_label = QtGui.QLabel("Trust region size")
        # max_trust_region_label = QtGui.QLabel("Max trust region size")
        # trust_box_size = QtGui.QSpinBox(self)
        # trust_box_size.setValue(self.sqp_config["trust_region_size"])
        # max_trust_region = QtGui.QSpinBox(self)
        # max_trust_region.setValue(self.sqp_config["max_trust_region_size"])
        # hbox.addLayout(sqp_grid)
        # hbox.addLayout(grid1)
        window = QtGui.QGroupBox(self)
        window.setTitle("SQP solver Parameters")

        robot_labels = []
        # robot_combo_box = []
        robot_grid = QtGui.QGridLayout()
        self.robot_combo_box =QtGui.QComboBox(self)
        robot_label = QtGui.QLabel("Robot Planning Group")

        # Definition des Tracing Parameters widgets
        for i in range(len(self.robot_config["groups"])):
            for key, value in self.robot_config["groups"][i].iteritems():
                if key == "name":
                    # robot_labels.append({value: QtGui.QLabel(value)})
                    self.robot_combo_box.addItem(value)
                    # robot_grid.append({value: QtGui.QGridLayout()})

        hbox.addLayout(robot_grid)
        robot_grid.addWidget(robot_label, 2, 0)
        robot_grid.addWidget(self.robot_combo_box, 2, 1)

        self.robot_combo_box.currentIndexChanged.connect(self.selectionchange)


        window.setLayout(hbox)



        self.setGeometry(300, 300, 850, 300)
        self.setWindowTitle('Trajectory Planner')

        self.show()

    def change_value1(self, val):
       print val
    def selectionchange(self, i):

        print "Current index", i, "selection changed ", self.robot_combo_box.currentText()

def main():
    app = QtGui.QApplication(sys.argv)
    ex = PlannerGui()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()