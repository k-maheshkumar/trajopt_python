import sys
from PyQt4 import QtGui, QtCore
import yaml
import numpy as np
from scripts.utils import yaml_paser as yaml


class PlannerGui(QtGui.QWidget):
    def __init__(self):
        super(Example, self).__init__()

        file_path_prefix = '../../config/'
        sqp_config_file = file_path_prefix + 'sqp_config.yaml'

        sqp_yaml = yaml.ConfigParser(sqp_config_file)
        self.sqp_config = sqp_yaml.get_by_key("sqp")
        robot_config_file = file_path_prefix + 'robot_config.yaml'
        robot_yaml = yaml.ConfigParser(robot_config_file)
        self.robot_config = robot_yaml.get_by_key("robot")
        print "in init", self.robot_config
        self.initUI()

    def initUI(self):
        hbox = QtGui.QHBoxLayout()
        grid = QtGui.QGridLayout()
        grid1 = QtGui.QGridLayout()

        # Definition des Tracing Parameters widgets
        trust_box_size_label = QtGui.QLabel("Trust region size")
        max_trust_region_label = QtGui.QLabel("Max trust region size")
        trust_box_size = QtGui.QSpinBox(self)
        trust_box_size.setValue(self.sqp_config["trust_region_size"])
        max_trust_region = QtGui.QSpinBox(self)
        max_trust_region.setValue(self.sqp_config["max_trust_region_size"])
        window = QtGui.QGroupBox(self)
        window.setTitle("SQP solver Parameters")
        hbox.addLayout(grid)
        hbox.addLayout(grid1)

        robot_group_label = QtGui.QLabel("Robot Planning Group")
        self.robot_group_combo = QtGui.QComboBox(self)
        temp = [x["name"] for x in self.robot_config["groups"]]
        temp1 = [self.robot_config["group"][k]["joints"] for k in range(len(x)) for x in self.robot_config["groups"] if
                 "sub-groups" in x]
        print "temp", temp1

        # temp1 = [temp[y] for y in range(len(x))x for x in temp1]
        flattened_list = [y for x in temp1 for y in x]

        print "temp", flattened_list
        self.robot_group_combo.addItems(temp)
        self.robot_group_combo.addItems(flattened_list)
        self.robot_group_combo.currentIndexChanged.connect(self.selectionchange)
        grid.addWidget(trust_box_size_label, 0, 0)
        grid.addWidget(trust_box_size, 0, 1)
        grid.addWidget(max_trust_region_label, 1, 0)
        grid.addWidget(max_trust_region, 1, 1)

        grid1.addWidget(robot_group_label, 0, 2)
        grid1.addWidget(self.robot_group_combo, 0, 3)

        window.setLayout(hbox)

        # self.setLayout(hbox)


        self.setGeometry(300, 300, 550, 300)
        self.setWindowTitle('Trajectory Planner')

        self.show()

    def selectionchange(self, i):
        print "Items in the list are :"

        # for count in range(self.cb.count()):
        #     print self.cb.itemText(count)
        print "Current index", i, "selection changed ", self.robot_group_combo.currentText()


def main():
    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()