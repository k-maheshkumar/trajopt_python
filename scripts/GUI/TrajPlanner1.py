from PyQt4 import QtGui
from scripts.utils import yaml_paser as yaml
import functools
from scripts.simulation import SimulationWorld

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
        self.start_simulation_button = QtGui.QPushButton('Start Simulation')

        self.sqp_labels = {}
        self.sqp_spin_box = {}
        self.sqp_combo_box = {}

        self.sqp_config_group_box = QtGui.QGroupBox('SQP solver Parameters')
        self.sqp_config_form = QtGui.QFormLayout()

        self.sqp_config_scroll = QtGui.QScrollArea()

        self.robot_config_params = {}
        self.robot_config_labels = {}
        self.robot_config_spin_box = {}
        self.robot_config_combo_box = {}
        self.robot_config_params_spin_box = {}
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

        self.sim_world = SimulationWorld.SimulationWorld()

        self.selected_robot_combo_value = {}
        self.selected_robot_spin_value = {}


        self.initUI(25)

    def initUI(self, val):

        self.main_hbox_layout = QtGui.QHBoxLayout()
        self.main_layout = QtGui.QVBoxLayout(self)

        self.main_layout.addLayout(self.main_hbox_layout)
        self.main_layout.addStretch(1)

        min_sqp_config = 0.00001
        max_sqp_config = 100
        min_robot_config = 3
        max_robot_config = 30


        for key, value in self.sqp_config.items():
            self.sqp_labels[key] = QtGui.QLabel(key)
            if not isinstance(value, list):
                self.sqp_spin_box[key] = QtGui.QDoubleSpinBox(self)

            else:
                self.sqp_combo_box[key] = QtGui.QComboBox(self)

        for key in self.sqp_labels:
            if not isinstance(self.sqp_config[key], list):
                self.sqp_config_form.addRow(self.sqp_labels[key], self.sqp_spin_box[key])
                self.sqp_spin_box[key].setValue(float(self.sqp_config[key]))
                self.sqp_spin_box[key].valueChanged.connect(functools.partial(self.on_sqp_spin_box_value_changed, key))
                if abs(float(self.sqp_config[key])) < 1:
                    self.sqp_spin_box[key].setSingleStep(0.01)
                if float(self.sqp_config[key]) < 0:
                    self.sqp_spin_box[key].setRange(-max_sqp_config, max_sqp_config)
                if float(self.sqp_config[key]) > 100:
                    self.sqp_spin_box[key].setRange(-max_sqp_config, max_sqp_config * 1e4)
            else:
                self.sqp_config_form.addRow(self.sqp_labels[key], self.sqp_combo_box[key])
                self.sqp_combo_box[key].addItems(self.sqp_config[key])
                self.sqp_combo_box[key].currentIndexChanged.connect(
                    functools.partial(self.on_sqp_combo_box_value_changed, key))

                self.sqp_config_group_box.setLayout(self.sqp_config_form)

        self.sqp_config_scroll.setWidget(self.sqp_config_group_box)
        self.sqp_config_scroll.setWidgetResizable(True)
        self.sqp_config_scroll.setFixedHeight(400)
        self.main_hbox_layout.addWidget(self.sqp_config_scroll)

        self.robot_config_hbox.addItem(self.robot_config_form)
        self.robot_config_hbox.addStretch(1)
        self.robot_config_vbox_layout.addItem(self.robot_config_hbox)

        for key in self.robot_config:
            # print key, value
            if key != "config_params":
                self.robot_config_combo_box[key] = QtGui.QComboBox(self)
                self.robot_config_combo_box[key].addItem("Select")
                self.robot_config_form.addRow(key, self.robot_config_combo_box[key])
                self.robot_config_combo_box[key].currentIndexChanged.connect(
                    functools.partial(self.on_robot_config_combo_box_value_changed, key))
                for key1, value1 in self.robot_config[key].items():
                    self.robot_config_combo_box[key].addItem(key1)
                    # self.selected_robot_combo_value[key] = value1
                    # print " key1, value1",  key1, value1

            else:
                for key1, value1 in self.robot_config[key].items():
                    # print " key1, value1",  key1, value1
                    self.robot_config_params_spin_box[key1] = QtGui.QDoubleSpinBox(self)
                    self.robot_config_form.addRow(key1, self.robot_config_params_spin_box[key1])
                    self.robot_config_params_spin_box[key1].setValue(float(value1))
                    self.robot_config_params_spin_box[key1].valueChanged.connect(functools.partial(self.on_robot_spin_box_value_changed, key1))
                    self.robot_config_params_spin_box[key1].setRange(min_robot_config, max_robot_config)
                    # self.selected_robot_spin_value[key] = value1

        self.robot_action_button_hbox.addStretch(1)

        for key in self.robot_action_buttons:
            if key != "config_params":
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
        self.simulation_scroll.setWidget(self.start_simulation_button)
        self.start_simulation_button.clicked.connect(self.on_start_simulation_clicked)
        self.simulation_scroll.setWidgetResizable(True)
        self.simulation_scroll.setFixedHeight(400)
        self.main_hbox_layout.addWidget(self.simulation_scroll)

    def on_start_simulation_clicked(self):
        self.sim_world.run_simulation()
        self.is_simulation_started = True

    def on_robot_spin_box_value_changed(self, key, value):
        # print(key, value)
        # self.selected_robot_spin_value.clear()
        self.selected_robot_spin_value[str(key)] = value
        # print self.selected_robot_spin_value[key]

    def on_robot_action_button_clicked(self, key):
        if not self.selected_robot_spin_value:
            for spin_box_key in self.robot_config_params_spin_box:
                self.selected_robot_spin_value[spin_box_key] = self.robot_config_params_spin_box[spin_box_key].value()
        # print "self.selected_robot_combo_value", self.selected_robot_combo_value
        # print "self.selected_robot_spin_value", self.selected_robot_spin_value
        if key == "plan":
            if "samples" in self.selected_robot_spin_value:
                samples = self.selected_robot_spin_value["samples"]
            if "duration" in self.selected_robot_spin_value:
                duration = self.selected_robot_spin_value["duration"]
            if "joints_groups" in self.selected_robot_combo_value:
                group = self.selected_robot_combo_value["joints_groups"]
            if "joint_configurations" in self.selected_robot_combo_value:
                goal_state = self.selected_robot_combo_value["joint_configurations"]
            self.sim_world.plan_trajectory(group, goal_state, samples, duration)
    def on_sqp_spin_box_value_changed(self, key, value):
        # print (key, value)
        self.sqp_config[str(key)] = value
        self.sqp_yaml.save()

    def on_sqp_combo_box_value_changed(self, combo_box_key):
        print (combo_box_key, self.sqp_combo_box[combo_box_key].currentText())

    def on_robot_config_combo_box_value_changed(self, combo_box_key):
        # self.selected_robot_combo_value.clear()
        # print (combo_box_key, self.robot_config_combo_box[combo_box_key].currentText())
        key = self.robot_config_combo_box[combo_box_key].currentText()
        if key != "Select":
            # print self.robot_config_combo_box[combo_box_key][self.robot_config_combo_box[combo_box_key].currentText()]
            self.selected_robot_combo_value[str(combo_box_key)] = self.robot_config[combo_box_key][str(key)]
        # print self.selected_robot_combo_value