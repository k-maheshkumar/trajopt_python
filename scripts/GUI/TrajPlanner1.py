from PyQt4 import QtGui
from scripts.utils import yaml_paser as yaml
import functools


class PlannerGui(QtGui.QWidget):
    def __init__(self, val):
        QtGui.QWidget.__init__(self)

        file_path_prefix = '../../config/'
        sqp_config_file = file_path_prefix + 'sqp_config.yaml'

        sqp_yaml = yaml.ConfigParser(sqp_config_file)
        self.sqp_config = sqp_yaml.get_by_key("sqp")
        robot_config_file = file_path_prefix + 'robot_config.yaml'
        robot_yaml = yaml.ConfigParser(robot_config_file)
        self.robot_config = robot_yaml.get_by_key("robot")

        self.initUI(25)

    def initUI(self, val):

        grid_layout = QtGui.QGridLayout()

        main_layout = QtGui.QVBoxLayout(self)

        main_layout.addLayout(grid_layout)

        self.sqp_labels = {}
        self.sqp_spin_box = {}
        self.sqp_combo_box = {}
        # self.sqp_grid = QtGui.QGridLayout()

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

        grid = QtGui.QGridLayout()
        sqp_config_group_box = QtGui.QGroupBox('SQP solver Parameters')
        sqp_config_form = QtGui.QFormLayout()

        for key, value in self.sqp_labels.iteritems():
            if not isinstance(self.sqp_config[key], list):
                sqp_config_form.addRow(self.sqp_labels[key], self.sqp_spin_box[key])
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
                sqp_config_form.addRow(self.sqp_labels[key], self.sqp_combo_box[key])
                self.sqp_combo_box[key].addItems(self.sqp_config[key])
                self.sqp_combo_box[key].currentIndexChanged.connect(
                    functools.partial(self.on_combo_box_value_changed, key))


        # for i in range(val):
        #     labellist.append(QtGui.QLabel('mylabel'))
        #     combolist.append(QtGui.QComboBox())
        #     myform.addRow(labellist[i], combolist[i])
        sqp_config_group_box.setLayout(sqp_config_form)
        sqp_config_scroll = QtGui.QScrollArea()
        sqp_config_scroll.setWidget(sqp_config_group_box)
        sqp_config_scroll.setWidgetResizable(True)
        sqp_config_scroll.setFixedHeight(400)
        grid_layout.addWidget(sqp_config_scroll, 0, 0)

        self.robot_config_labels = {}
        self.robot_config_spin_box = {}
        self.robot_config_combo_box = {}
        robot_config_group_box = QtGui.QGroupBox('Robot Config Parameters')
        robot_config_form = QtGui.QFormLayout()
        for key,value in self.robot_config["joints"].iteritems():
            self.robot_config_combo_box[key] = QtGui.QComboBox(self)
            self.robot_config_labels[key] = QtGui.QLabel(key)

            for key1,value1 in value.iteritems():
                print key1, value1
            #     self.robot_config_labels[key] = QtGui.QLabel(key)
            #     self.robot_config_combo_box[key] = QtGui.QComboBox(self)
            #
                robot_config_form.addRow(self.robot_config_labels[key], self.robot_config_combo_box[key])
                self.robot_config_combo_box[key].addItem(key1)



        robot_config_group_box.setLayout(robot_config_form)
        robot_config_scroll = QtGui.QScrollArea()
        robot_config_scroll.setWidget(robot_config_group_box)
        robot_config_scroll.setWidgetResizable(True)
        robot_config_scroll.setFixedHeight(400)
        grid_layout.addWidget(robot_config_scroll, 0, 1)



    def on_spin_box_value_changed(self, checked, message):
        print checked, message

    def on_combo_box_value_changed(self, combo_box_key):
        print combo_box_key, self.sqp_combo_box[combo_box_key].currentText()


if __name__ == '__main__':
    import sys

    app = QtGui.QApplication(sys.argv)
    window = PlannerGui(25)
    window.setGeometry(200, 100, 800, 400)
    window.show()
    sys.exit(app.exec_())