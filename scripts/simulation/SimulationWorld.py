import pybullet as sim
from scripts.Planner import Robot
from scripts.interfaces.ISimulationWorldBase import ISimulationWorldBase
import time


class SimulationWorld():
    def __init__(self, file_path=None):
        location_prefix = '/home/mahe/masterThesis/bullet3/data/'
        # location_prefix = '/home/mahesh/libraries/bullet3/data/'
        urdf_file = "kuka_iiwa/model.urdf"
        self.robot = Robot.Robot(location_prefix + urdf_file)
        self.robot_id = -1
        self.joint_name_to_id = {}
        self.gui = sim.connect(sim.GUI)
        # self.gui = sim.connect(sim.DIRECT)
        self.no_of_joints = -1

        self.set_up_world(location_prefix, urdf_file)
        kukaEndEffectorIndex = 6
        useRealTimeSimulation = 1
        if useRealTimeSimulation:
            sim.setRealTimeSimulation(useRealTimeSimulation)
    def set_up_world(self, file_path, urdf_file):
        sim.loadURDF(file_path + "plane.urdf", [0, 0, -0.3], useFixedBase=True)
        sim.loadURDF(file_path + "table/table.urdf", [0, 0, -0.3], useFixedBase=True)

        self.robot_id = sim.loadURDF(file_path + urdf_file, [0, 0, 0], useFixedBase=True)
        sim.resetBasePositionAndOrientation(self.robot_id, [0, 0.25, 0.4], [0, 0, 0, 1])

        sim.setGravity(0, 0, -10)
        self.no_of_joints = sim.getNumJoints(self.robot_id)
        self.setup_jointId_to_jointName()


    def run_simulation(self):
        iteration_count = 0
        while 1:
            if iteration_count < 1:
                iteration_count += 1
                goal_state = {
                    'lbr_iiwa_joint_1': -2.0417782994426674,
                    'lbr_iiwa_joint_2': 0.9444594031189716,
                    'lbr_iiwa_joint_3': -1.591006403858707,
                    'lbr_iiwa_joint_4': -1.9222844444479184,
                    'lbr_iiwa_joint_5': 1.572303282659756,
                    'lbr_iiwa_joint_6': 1.5741716208788483,
                    'lbr_iiwa_joint_7': 1.5716145442929421
                }

                group1_test = ['lbr_iiwa_joint_1']

                group1 = ['lbr_iiwa_joint_1', 'lbr_iiwa_joint_2', 'lbr_iiwa_joint_3']
                group2 = ['lbr_iiwa_joint_4', 'lbr_iiwa_joint_5', 'lbr_iiwa_joint_6', 'lbr_iiwa_joint_7']
                duration = 6
                samples = 15

                self.plan_trajectory(group1, goal_state, samples, duration)
                # self.execute_trajectory(self.robot.get_trajectory(), samples)

    def plan_trajectory(self, group, goal_state, samples, duration, solver_config=None):
        # print group, goal_state, samples, duration
        # self.reset_joint_states(group)
        # current_state = self.get_current_states_for_given_joints(group)
        # print "current_state", current_state
        status = self.robot.plan_trajectory(group=group, current_state=self.get_current_states_for_given_joints(group),
                                   goal_state=goal_state, samples=int(samples), duration=int(duration),
                                   solver_config=solver_config)
        return status

    def get_current_states_for_given_joints(self, joints):
        current_state = {}
        for joint in joints:
            current_state[joint] = sim.getJointState(bodyUniqueId=self.robot_id, jointIndex=self.joint_name_to_id[joint])[0]
        return current_state

    def execute_trajectory(self):
        # print "len(trajectory)", len(trajectory)
        trajectories=self.robot.get_trajectory()
        for i in range(int(trajectories.no_of_samples)):
            for joint_name, corresponding_trajectory in trajectories.trajectory.items():
                sim.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.joint_name_to_id[joint_name],
                                          controlMode=sim.POSITION_CONTROL,
                                          targetPosition=corresponding_trajectory[i], targetVelocity=.1, force=500,
                                          positionGain=0.03,
                                          velocityGain=.5)
                # break
            # time.sleep(samples / float(duration))
            time.sleep(trajectories.duration / float(trajectories.no_of_samples))
        return "Trajectory execution has finished"

    def plan_and_execute_trajectory(self, group, goal_state, samples, duration, solver_config=None):
        status = "-1"
        status = self.plan_trajectory(group, goal_state, samples, duration, solver_config=None)
        status += " and "
        status += self.execute_trajectory()

        return status


    def setup_jointId_to_jointName(self):
        for i in range(self.no_of_joints):
            jointInfo = sim.getJointInfo(self.robot_id, i)
            # print jointInfo
            self.joint_name_to_id[jointInfo[1].decode('UTF-8')] = jointInfo[0]
        # print "joint_id_to_name", self.joint_id_to_name

    def reset_joint_states(self, joints, motor_dir=None, half_pi=None):
        if motor_dir is None:
            motor_dir = [-1, -1, -1, 1, 1, 1, 1]
        if half_pi is None:
            half_pi = 1.57079632679
        for joint in joints:
            for j in range(self.no_of_joints):
                sim.resetJointState(self.robot_id, self.joint_name_to_id[joint], motor_dir[j] * half_pi)

        return "Reset joints to random pose is complete"

#
# if __name__ == "__main__":
#     sim1 = SimulationWorld()
#     sim1.run_simulation()
