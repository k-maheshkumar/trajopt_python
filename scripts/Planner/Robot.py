from urdf_parser_py.urdf import URDF
import Planner
import time
from munch import *


class Robot:
    def __init__(self, urdf_file):

        location_prefix = '/home/mahesh/libraries/bullet3/data/'
        # location_prefix = '/home/mahe/masterThesis/bullet3/data/'

        self.model = URDF.from_xml_file(location_prefix + "kuka_iiwa/model.urdf")
        # self.__replace_joints_in_model_with_map()
        self.state = self.init_state()
        self.planner = {}

    def init_state(self):
        state = {}
        for joint in self.model.joints:
            state[joint.name] = {
                                    "current_value": 0
                                }
        state = munchify(state)
        print state.lbr_iiwa_joint_5.current_value
        return state

    def get_state(self):
        return self.state

    def get_trajectory(self):
        return self.planner

    def __replace_joints_in_model_with_map(self):
        joints = {}
        for joint in self.model.joints:
            joints[joint.name] = joint
        del self.model.joints[:]
        self.model.joints = joints

    def plan_trajectory1(self, group, states, samples, duration):
        joints = []
        # for name, joint in self.joints.iteritems():
        for joint_in_group in group:
            if joint_in_group in self.model.joints:
                joints.append(self.model.joints[joint_in_group])
                # joints.append({"name": self.model.joints[joint_in_group].name, "states": states[joint_in_group], "limits": {self.model.joints[joint_in_group].limit})

        print joints[0]
        self.planner = Planner.TrajectoryOptimizationPlanner(joints=joints, samples=samples, duration=duration,
                                                             solver="SCS", solver_class=1, decimals_to_round=2)
        # self.trajectory.displayProblem()

    def plan_trajectory(self, joint_group, states, samples, duration):
        joints = []
        for joint in self.model.joints:
            for joint_in_group in joint_group:
                if joint.name == joint_in_group:
                    joints.append(munchify({
                        "name": joint.name,
                        "states": states[joint_in_group],
                        "limits": joint.limit
                    }))
        self.planner = Planner.TrajectoryOptimizationPlanner(joints=joints, samples=samples, duration=duration,
                                                             solver="SCS", solver_class=1, decimals_to_round=2)
        self.planner.calculate_trajectory()
        # self.trajectory.displayProblem()

    def get_robot_trajectory(self):
        return self.planner.trajectory.get_trajectory()

    def update_robot_state(self):
        pass


if __name__ == "__main__":
    robo = Robot(None)

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

    group1_test = ['lbr_iiwa_joint_1']

    group1 = ['lbr_iiwa_joint_1', 'lbr_iiwa_joint_2', 'lbr_iiwa_joint_3']
    group2 = ['lbr_iiwa_joint_4', 'lbr_iiwa_joint_5', 'lbr_iiwa_joint_6', 'lbr_iiwa_joint_7']
    duration = 6
    samples = 5
    # robo.plan_trajectory(group1_test, states, samples, duration)
    # joints.plan_trajectory(group1 , samples, duration)
    # print robo.model.joints["lbr_iiwa_joint_5"]
    start = time.time()
    # print robo.get_robot_trajectory()
    end = time.time()
    print("computation time: ", end - start)