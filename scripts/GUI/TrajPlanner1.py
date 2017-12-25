from PyQt4 import QtGui
from scripts.utils import yaml_paser as yaml
import functools
import sys


class PlannerGui(QtGui.QWidget):
    def __init__(self, val):
        QtGui.QWidget.__init__(self)
        self.setGeometry(200, 100, 1200, 500)
        file_path_prefix = '../../config/'
        self.sqp_config_file = file_path_prefix + 'sqp_config.yaml'

        self.sqp_yaml = yaml.ConfigParser(self.sqp_config_file)
        self.sqp_config = self.sqp_yaml.get_by_key("sqp")
        robot_config_file = file_path_prefix + 'robot_config.yaml'
        robot_yaml = yaml.ConfigParser(robot_config_file)
        self.robot_config = robot_yaml.get_by_key("robot")

        self.sqp_labels = {}
        self.sqp_spin_box = {}
        self.sqp_combo_box = {}

        self.sqp_config_group_box = QtGui.QGroupBox('SQP solver Parameters')
        self.sqp_config_form = QtGui.QFormLayout()

        self.sqp_config_scroll = QtGui.QScrollArea()

        self.robot_config_labels = {}
        self.robot_config_spin_box = {}
        self.robot_config_combo_box = {}
        self.robot_config_group_box = QtGui.QGroupBox('Robot Actions')

        self.robot_config_hbox = QtGui.QVBoxLayout()

        self.robot_config_vbox_layout = QtGui.QVBoxLayout()

        self.robot_config_form = QtGui.QFormLayout()

        self.robot_action_buttons = {}
        self.robot_action_button_group = QtGui.QButtonGroup(self)
        self.robot_action_buttons["execute"] = QtGui.QPushButton('Execute')
        self.robot_action_buttons["plan"] = QtGui.QPushButton('Plan')

        self.robot_action_button_hbox = QtGui.QHBoxLayout()

        self.robot_config_scroll = QtGui.QScrollArea()

        self.simulation_scroll = QtGui.QScrollArea()

        self.initUI(25)

    def initUI(self, val):

        self.main_hbox_layout = QtGui.QHBoxLayout()
        self.main_layout = QtGui.QVBoxLayout(self)

        self.main_layout.addLayout(self.main_hbox_layout)
        self.main_layout.addStretch(1)

        min_ = 0.00001
        max_ = 100

        for key, value in self.sqp_config.items():
            self.sqp_labels[key] = QtGui.QLabel(key)
            if not isinstance(value, list):
                self.sqp_spin_box[key] = QtGui.QDoubleSpinBox(self)

            else:
                self.sqp_combo_box[key] = QtGui.QComboBox(self)

        for key, value in self.sqp_labels.items():
            if not isinstance(self.sqp_config[key], list):
                self.sqp_config_form.addRow(self.sqp_labels[key], self.sqp_spin_box[key])
                self.sqp_spin_box[key].setValue(float(self.sqp_config[key]))
                self.sqp_spin_box[key].valueChanged.connect(functools.partial(self.on_spin_box_value_changed, key))
                if abs(float(self.sqp_config[key])) < 1:
                    self.sqp_spin_box[key].setSingleStep(0.01)
                if float(self.sqp_config[key]) < 0:
                    self.sqp_spin_box[key].setRange(-max_, max_)
                if float(self.sqp_config[key]) > 100:
                    self.sqp_spin_box[key].setRange(-max_, max_ * 1e4)
            else:
                self.sqp_config_form.addRow(self.sqp_labels[key], self.sqp_combo_box[key])
                self.sqp_combo_box[key].addItems(self.sqp_config[key])
                self.sqp_combo_box[key].currentIndexChanged.connect(
                    functools.partial(self.on_combo_box_value_changed, key))

                self.sqp_config_group_box.setLayout(self.sqp_config_form)

        self.sqp_config_scroll.setWidget(self.sqp_config_group_box)
        self.sqp_config_scroll.setWidgetResizable(True)
        self.sqp_config_scroll.setFixedHeight(400)
        self.main_hbox_layout.addWidget(self.sqp_config_scroll)


        self.robot_config_hbox.addItem(self.robot_config_form)
        self.robot_config_hbox.addStretch(1)
        self.robot_config_vbox_layout.addItem(self.robot_config_hbox)

        for key, value in self.robot_config.items():
            # print key, value
            self.robot_config_combo_box[key] = QtGui.QComboBox(self)
            self.robot_config_form.addRow(key, self.robot_config_combo_box[key])
            self.robot_config_combo_box[key].currentIndexChanged.connect(
                functools.partial(self.on_robot_config_combo_box_value_changed, key))
            for key1, value1 in self.robot_config[key].items():
                self.robot_config_combo_box[key].addItem(key1)


        self.robot_action_button_hbox.addStretch(1)

        for key, button in self.robot_action_buttons.items():
            self.robot_action_button_hbox.addWidget(self.robot_action_buttons[key])
            self.robot_action_buttons[key].clicked.connect(
                functools.partial(self.on_robot_action_button_clicked, key))
            self.robot_action_buttons[key].setMaximumWidth(220)

        self.robot_config_vbox_layout.addItem(self.robot_action_button_hbox)

        self.robot_config_vbox_layout.addStretch(1)

        self.robot_config_group_box.setLayout(self.robot_config_vbox_layout)
        self.robot_config_scroll.setWidget(self.robot_config_group_box)
        self.robot_config_scroll.setWidgetResizable(True)
        self.robot_config_scroll.setFixedHeight(400)
        self.main_hbox_layout.addWidget(self.robot_config_scroll)

        self.simulation_scroll.setWidget(QtGui.QPushButton('Start Simulation'))
        self.simulation_scroll.setWidgetResizable(True)
        self.simulation_scroll.setFixedHeight(400)
        self.main_hbox_layout.addWidget(self.simulation_scroll)

    def on_robot_action_button_clicked(self, key):
        print ("clicked button: ", key)

    def on_spin_box_value_changed(self, key, value):
        print (key, value)
        self.sqp_config[key] = value
        self.sqp_yaml.save()

    def on_combo_box_value_changed(self, combo_box_key):
        print (combo_box_key, self.sqp_combo_box[combo_box_key].currentText())

    def on_robot_config_combo_box_value_changed(self, combo_box_key):
        print (combo_box_key, self.robot_config_combo_box[combo_box_key].currentText())
        key = self.robot_config_combo_box[combo_box_key].currentText()
        print (self.robot_config[combo_box_key][str(key)])
        # print self.robot_config_combo_box[combo_box_key][self.robot_config_combo_box[combo_box_key].currentText()]


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    window = PlannerGui(25)
    window.show()
    sys.exit(app.exec_())