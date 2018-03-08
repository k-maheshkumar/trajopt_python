from PyQt4 import QtGui
from scripts.utils import yaml_paser as yaml
import functools
from scripts.simulation.SimulationWorld import SimulationWorld
import numpy as np
import logging
import collections

class PlannerGui(QtGui.QMainWindow):
    def __init__(self, verbose=False, file_log=False, simulation=None):
        QtGui.QMainWindow.__init__(self)
        self.setGeometry(200, 100, 1200, 400)

        file_path_prefix = '../../config/'
        self.default_config = yaml.ConfigParser(file_path_prefix + 'default_config.yaml')
        self.config = self.default_config.get_by_key("config")

        self.sqp_config_file = file_path_prefix + self.config["solver"]

        self.sqp_yaml = yaml.ConfigParser(self.sqp_config_file)
        self.sqp_config = self.sqp_yaml.get_by_key("sqp")
        robot_config_file = file_path_prefix + self.config["robot"]
        robot_yaml = yaml.ConfigParser(robot_config_file)
        self.robot_config = robot_yaml.get_by_key("robot")
        self.start_simulation_button = QtGui.QPushButton('Start Simulation')

        self.sqp_labels = collections.
        self.sqp_spin_box = collections.OrderedDict()
        self.sqp_combo_box = collections.OrderedDict()

        self.sqp_config_group_box = QtGui.QGroupBox('SQP solver Parameters')
        self.sqp_config_form = QtGui.QFormLayout()

        self.sqp_config_scroll = QtGui.QScrollArea()

        self.robot_config_params = collections.OrderedDict()
        self.robot_config_labels = collections.OrderedDict()
        self.robot_config_spin_box = collections.OrderedDict()
        self.robot_config_combo_box = collections.OrderedDict()
        self.robot_config_params_spin_box = collections.OrderedDict()
        self.robot_config_group_box = QtGui.QGroupBox('Robot Actions')

        self.robot_config_hbox = QtGui.QVBoxLayout()

        self.robot_config_vbox_layout = QtGui.QVBoxLayout()

        self.robot_config_form = QtGui.QFormLayout()

        self.robot_action_buttons = collections.OrderedDict()
        self.robot_action_button_group = QtGui.QButtonGroup(self)
        self.robot_action_buttons["execute"] = QtGui.QPushButton('Execute')
        self.robot_action_buttons["plan"] = QtGui.QPushButton('Plan')
        self.robot_action_buttons["random_pose"] = QtGui.QPushButton('Random Pose')
        self.robot_action_buttons["plan_and_execute"] = QtGui.QPushButton('Plan and Execute')

        self.robot_action_button_hbox = QtGui.QHBoxLayout()

        self.robot_config_scroll = QtGui.QScrollArea()

        self.simulation_scroll = QtGui.QScrollArea()

        self.selected_robot_combo_value = collections.OrderedDict()
        self.selected_robot_spin_value = collections.OrderedDict()

        self.statusBar = QtGui.QStatusBar()

        self.main_widget = QtGui.QWidget(self)
        self.main_layout = QtGui.QVBoxLayout(self.main_widget)
        self.main_hbox_layout = QtGui.QHBoxLayout()

        self.last_status = "Last Status: "
        self.init_ui(25)
        # self.sim_world = SimulationWorld.SimulationWorld(self.robot_config["urdf"])
        self.sim_world = simulation
        self.can_execute_trajectory = False
        main_logger_name = "Trajectory_Planner"
        self.logger = logging.getLogger(main_logger_name)
        self.setup_logger(main_logger_name, verbose, file_log)

    def setup_logger(self, main_logger_name, verbose=False, log_file=False):

        # creating a formatter
        formatter = logging.Formatter('-%(asctime)s - %(name)s - %(levelname)-8s: %(message)s')

        # create console handler with a debug log level
        log_console_handler = logging.StreamHandler()
        if log_file:
            # create file handler which logs info messages
            logger_file_handler = logging.FileHandler(main_logger_name + '.log', 'w', 'utf-8')
            logger_file_handler.setLevel(logging.INFO)
            # setting handler format
            logger_file_handler.setFormatter(formatter)
            # add the file logging handlers to the logger
            self.logger.addHandler(logger_file_handler)

        if verbose == "WARN":
            self.logger.setLevel(logging.WARN)
            log_console_handler.setLevel(logging.WARN)

        elif verbose == "INFO" or verbose is True:
            self.logger.setLevel(logging.INFO)
            log_console_handler.setLevel(logging.INFO)

        elif verbose == "DEBUG":
            self.logger.setLevel(logging.DEBUG)
            log_console_handler.setLevel(logging.DEBUG)

        # setting console handler format
        log_console_handler.setFormatter(formatter)
        # add the handlers to the logger
        self.logger.addHandler(log_console_handler)

    def init_ui(self, val):

        self.main_widget.setLayout(self.main_layout)
        self.main_layout.addLayout(self.main_hbox_layout)
        self.main_layout.addStretch(1)
        self.setCentralWidget(self.main_widget)
        self.setStatusBar(self.statusBar)

        min_sqp_config = 0.00001
        max_sqp_config = 100
        min_robot_config = 3
        max_robot_config = 60

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
            if key != "config_params" and key != "urdf":
                self.robot_config_combo_box[key] = QtGui.QComboBox(self)
                self.robot_config_combo_box[key].addItem("Select")
                self.robot_config_form.addRow(key, self.robot_config_combo_box[key])
                self.robot_config_combo_box[key].currentIndexChanged.connect(
                    functools.partial(self.on_robot_config_combo_box_value_changed, key))
                for key1, value1 in self.robot_config[key].items():
                    self.robot_config_combo_box[key].addItem(key1)
                    # self.selected_robot_combo_value[key] = value1
                    # print " key1, value1",  key1, value1

            if key == "config_params":
                for key1, value1 in self.robot_config[key].items():
                    print " key1, value1", key1, value1
                    self.robot_config_params_spin_box[key1] = QtGui.QDoubleSpinBox(self)
                    self.robot_config_form.addRow(key1, self.robot_config_params_spin_box[key1])
                    self.robot_config_params_spin_box[key1].setValue(float(value1["value"]))
                    self.robot_config_params_spin_box[key1].valueChanged.connect(
                        functools.partial(self.on_robot_spin_box_value_changed, key1))
                    self.robot_config_params_spin_box[key1].setRange(value1["min"], value1["max"])
                    self.robot_config_params_spin_box[key1].setSingleStep(value1["step"])

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
        # self.sim_world.run_simulation()
        self.is_simulation_started = True

    def on_robot_spin_box_value_changed(self, key, value):
        # print(key, value)
        # self.selected_robot_spin_value.clear()
        self.selected_robot_spin_value[str(key)] = value
        # print self.selected_robot_spin_value[key]

    def on_robot_action_button_clicked(self, key):
        # if not self.selected_robot_spin_value:
        for spin_box_key in self.robot_config_params_spin_box:
            self.selected_robot_spin_value[spin_box_key] = self.robot_config_params_spin_box[spin_box_key].value()
        # print "self.selected_robot_combo_value", self.selected_robot_combo_value
        # print "self.selected_robot_spin_value", self.selected_robot_spin_value

        samples = None
        duration = None
        group = None
        safe_collision_distance = None
        goal_state = None
        if "samples" in self.selected_robot_spin_value:
            samples = self.selected_robot_spin_value["samples"]
        if "duration" in self.selected_robot_spin_value:
            duration = self.selected_robot_spin_value["duration"]
        if "joints_groups" in self.selected_robot_combo_value:
            group = self.selected_robot_combo_value["joints_groups"]
        if "joint_configurations" in self.selected_robot_combo_value:
            goal_state = self.selected_robot_combo_value["joint_configurations"]
        if "safe_collision_distance" in self.selected_robot_spin_value:
            safe_collision_distance = self.selected_robot_spin_value["safe_collision_distance"]
        # if "upper_collision_limit" in self.selected_robot_spin_value:
        #     upper_collision_limit = self.selected_robot_spin_value["upper_collision_limit"]
        # print samples
        # print duration
        # print group
        # print goal_state
        # print safe_collision_distance

        if group is not None and goal_state is not None:
            if samples is not None and duration is not None and self.sqp_config is not None and \
                            safe_collision_distance is not None:
                print "here"
                if key == "plan" or key == "plan_and_execute":
                    self.statusBar.clearMessage()
                    status = "Please wait, trajectory is being planned... Then trajectory will be executed.."
                    self.statusBar.showMessage(self.last_status + status)
                    status = self.initiate_plan_trajectory(group, goal_state, samples, duration,
                                                           safe_collision_distance)
                    self.statusBar.clearMessage()
                    self.statusBar.showMessage(self.last_status + status)
                if key == "plan_and_execute" or key == "execute":
                    if self.can_execute_trajectory:
                        self.statusBar.clearMessage()
                        status = "Please wait, trajectory is being executed.."
                        self.statusBar.showMessage(self.last_status + status)
                        status = self.sim_world.execute_trajectory()
                        self.statusBar.clearMessage()
                        self.statusBar.showMessage(self.last_status + status)
                    else:
                        self.statusBar.clearMessage()
                        status = "First plan the trajectory to execute.."
                        self.statusBar.showMessage(self.last_status + status)
        else:
            self.statusBar.clearMessage()
            status = "Please select the planning group and joint configuration.."
            self.statusBar.showMessage(self.last_status + status)

        if key == "random_pose":
            self.statusBar.clearMessage()
            status = "Please wait, random pose for the joints are being set.."
            self.statusBar.showMessage(self.last_status + status)
            if group is not None:
                status = self.sim_world.reset_joint_states(group, motor_dir=np.random.rand(1, len(group)).flatten())
                self.statusBar.showMessage(self.last_status + status)
            else:
                self.statusBar.clearMessage()
                status = "Please select the planning group.."
                self.statusBar.showMessage(self.last_status + status)


    def initiate_plan_trajectory(self, group, goal_state, samples, duration, collision_d_safe_limit):
        can_execute_trajectory = False
        if samples is not None and duration is not None \
                and group is not None and goal_state is not None and self.sqp_config is not None:
            status, self.can_execute_trajectory = self.sim_world.plan_trajectory(group=group, goal_state=goal_state, samples=samples,
                                                                                 duration=duration,
                                                                                 collision_safe_distance=collision_d_safe_limit,
                                                                                 solver_config=self.sqp_config, )
        else:
            status = "Please select the planning group and joint configuration.."

        return status

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