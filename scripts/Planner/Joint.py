from urdf_parser_py.urdf import Joint as urdf_joint
from urdf_parser_py.urdf import URDF
import Planner
import time

class Joint(urdf_joint):
    def __init__(self, joint, start, end):
        from urdf_parser_py.urdf import URDF

        # print robot.joint_map[joint.name]
        urdf_joint.__init__(self, name=joint.name, parent=joint.parent, child=joint.child, joint_type=joint.joint_type,
                            axis=joint.axis, origin=joint.origin,
                            limit=joint.limit, dynamics=joint.dynamics, safety_controller=joint.safety_controller,
                            calibration=joint.calibration, mimic=joint.mimic)

        self.start = start
        self.end = end


class Joints:
    def __init__(self, joints):
        self.joints = {}
        self.trajectory = []
        for joint in joints:
            self.joints[joint.name] = Joint(joint, 1, 2)

    def update_states(self, states):
        decimal_places = 4
        for key, value in states.items():
            self.joints[key].start = round(value['start'], decimal_places)
            self.joints[key].end = round(value['end'], decimal_places)
            # print self.joints[key].name
            # print self.joints[key].limit
            # print self.joints[key].start, self.joints[key].end

    def plan_trajectory(self, group, samples, duration):
        joints = []
        # for name, joint in self.joints.iteritems():
        for joint_in_group in group:
            if joint_in_group in self.joints:
                joints.append(self.joints[joint_in_group])
        print len(joints)
        self.trajectory = Planner.TrajectoryOptimizationPlanner(joints=joints, samples=samples, duration=duration,
                                                                solver="SCS", temp=0)
        self.trajectory.displayProblem()

    def get_trajectory(self):
        return self.trajectory.get_trajectory()


location_prefix = '/home/mahesh/libraries/bullet3/data/'
# location_prefix = '/home/mahe/masterThesis/bullet3/data/'

robot = URDF.from_xml_file(location_prefix + "kuka_iiwa/model.urdf")
#
startState = [-0.49197958189616936, 1.4223062659337982, -1.5688299779644697, -1.3135004031364736, 1.5696229411153653,
              1.5749627479696093, 1.5708037563007493]
endState = [-2.0417782994426674, 0.9444594031189716, -1.591006403858707, -1.9222844444479184, 1.572303282659756,
            1.5741716208788483, 1.5716145442929421]
# joints1 = []
# for joint in robot.joints:
#
#     joi = Joint(joint, 1, 2)
#     joints1.append(joi)
#     # print joi.name, joi.start, joi.end
#
# del robot.joints[:]
#
# for joint in joints1:
#     robot.joints.append(joint)
#
# for i in range(len(robot.joints)):
#     robot.joints[i].start = startState[i]
#     robot.joints[i].end = endState[i]
#
# plan = Planner.TrajectoryOptimizationPlanner(robot.joints, 10, 10)
# print plan.displayProblem()
# print plan.get_trajectory()


states = {
    # 'lbr_iiwa_joint_1': {"start": -0.49, "end": -2.0},
    'lbr_iiwa_joint_1': {"start": -0.49197958189616936, "end": -2.0417782994426674},
    'lbr_iiwa_joint_2': {"start": 1.4223062659337982, "end": 0.9444594031189716},
    'lbr_iiwa_joint_3': {"start": -1.5688299779644697, "end": -1.591006403858707},
    'lbr_iiwa_joint_4': {"start": -1.3135004031364736, "end": -1.9222844444479184},
    'lbr_iiwa_joint_5': {"start": 1.5696229411153653, "end": 1.572303282659756},
    'lbr_iiwa_joint_6': {"start": 1.5749627479696093, "end": 1.5741716208788483},
    'lbr_iiwa_joint_7': {"start": 1.5708037563007493, "end": 1.5716145442929421}
}
joints = Joints(robot.joints)
del robot.joints[:]
robot.joints = joints
joints.update_states(states)
group1_test = ['lbr_iiwa_joint_1']

group1 = ['lbr_iiwa_joint_1', 'lbr_iiwa_joint_2', 'lbr_iiwa_joint_3']
group2 = ['lbr_iiwa_joint_4', 'lbr_iiwa_joint_5', 'lbr_iiwa_joint_6', 'lbr_iiwa_joint_7']
duration = 6
samples = 5
# joints.plan_trajectory(group1, samples, duration)
joints.plan_trajectory(group1 + group2 + group1 + group2, samples, duration)
start = time.time()

print joints.get_trajectory()
end = time.time()
print("computation time: ", end - start)

# print robot.joints.get_trajectory()
