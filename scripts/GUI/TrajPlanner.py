import sys
from PyQt4 import QtGui, QtCore
import yaml
import numpy as np
from scripts.utils import yaml_paser as yaml
import itertools
import functools
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
        self.hbox = QtGui.QHBoxLayout()
        self.sqp_grid = QtGui.QGridLayout()
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


        self.sqp_labels = {}
        self.sqp_spin_box = {}
        self.sqp_combo_box = {}
        self.sqp_grid = QtGui.QGridLayout()

        min_ = 0.00001
        max_ = 100
        # Definition des Tracing Parameters widgets
        for key, value in self.sqp_config.iteritems():
            self.sqp_labels[key] = QtGui.QLabel(key)
            if not isinstance(value, list):
                self.sqp_spin_box[key] = QtGui.QDoubleSpinBox(self)

            else:
                # self.sqp_combo_box.append({key: QtGui.QComboBox(self)})
                self.sqp_combo_box[key] = QtGui.QComboBox(self)

        sqp_param_count = 0
        self.hbox.addLayout(self.sqp_grid)

        for key, value in self.sqp_labels.iteritems():
            self.sqp_grid.addWidget(self.sqp_labels[key], sqp_param_count, 0)

            # if self.sqp_config[key] is str:

            if not isinstance(self.sqp_config[key], list):
                self.sqp_grid.addWidget(self.sqp_spin_box[key], sqp_param_count, 1)
                self.sqp_spin_box[key].setValue(float(self.sqp_config[key]))
                # self.sqp_spin_box[key].setRange(min_, max_)
                self.sqp_spin_box[key].valueChanged.connect(functools.partial(self.on_spin_box_value_changed, key))

                if abs(float(self.sqp_config[key])) < 1:
                    self.sqp_spin_box[key].setSingleStep(0.01)
                if float(self.sqp_config[key]) < 0:
                    self.sqp_spin_box[key].setRange(-max_, max_)
                if float(self.sqp_config[key]) > 100:
                    print self.sqp_config[key]
                    self.sqp_spin_box[key].setRange(-max_, max_ * 1e4)

            else:
                self.sqp_grid.addWidget(self.sqp_combo_box[key], sqp_param_count, 1)
                self.sqp_combo_box[key].addItems(self.sqp_config[key])
                self.sqp_combo_box[key].currentIndexChanged.connect(functools.partial(self.on_combo_box_value_changed, key))

                # self.robot_combo_box.currentIndexChanged.connect(self.selectionchange)

            # self.connect(sqp_spin_box[key], QtCore.SIGNAL('valueChanged(double)'), self.change_value1)
            # sqp_spin_box[key].connect(QtCore.SIGNAL(self.test()), self.change_value1)

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
        # for i in range(len(self.robot_config["groups"])):
        #     for key, value in self.robot_config["groups"][i].iteritems():
        #         if key == "name":
        #             # robot_labels.append({value: QtGui.QLabel(value)})
        #             self.robot_combo_box.addItem(value)
        #             # robot_grid.append({value: QtGui.QGridLayout()})

        for key, value in self.robot_config.iteritems():
            for key1,value1 in self.robot_config[key].iteritems():
                print self.robot_config[key][key1]

        self.hbox.addLayout(robot_grid)
        robot_grid.addWidget(robot_label, 2, 0)
        robot_grid.addWidget(self.robot_combo_box, 2, 1)

        self.robot_combo_box.currentIndexChanged.connect(self.selectionchange)


        window.setLayout(self.hbox)



        self.setGeometry(300, 300, 850, 300)
        self.setWindowTitle('Trajectory Planner')

        self.show()

    def on_spin_box_value_changed(self, checked, message):
        print checked, message

    def   on_combo_box_value_changed(self, combo_box_key):
        print combo_box_key, self.sqp_combo_box[combo_box_key].currentText()
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