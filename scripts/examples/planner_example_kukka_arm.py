from scripts.simulation.SimulationWorld import SimulationWorld
import os
from scripts.Robot import Robot


class PlannerExample:
    def __init__(self):
        home = os.path.expanduser('~')

        location_prefix = home + '/masterThesis/bullet3/data/'

        urdf_file = location_prefix + "kuka_iiwa/model.urdf"
        self.robot = Robot.Robot(urdf_file)

        self.world = SimulationWorld(urdf_file, use_gui=True)
        self.world.set_gravity(0, 0, -10)
        self.world.toggle_rendering(0)
        self.world.set_robot(self.robot)
        self.world.load_urdf(urdf_file=location_prefix + "plane.urdf",
                             position=[0, 0, 0.0])

        table_id = self.world.load_urdf(urdf_file=location_prefix + "table/table.urdf",
                                        position=[0, 0, 0.0])

        self.world.robot_id = self.world.load_urdf(urdf_file, position=[0, 0.25, 0.6])
        # self.cylinder_id = self.create_constraint(shape=CYLINDER, height=0.70, radius=0.12,
        #                                           # position=[-0.17, -0.43, 0.9], mass=1)
        #                                           position=[-0.50, 0.42, 0.9], mass=100)
        self.box_id = self.world.create_constraint(shape=self.world.BOX, size=[0.1, 0.2, 0.45],
                                             # position=[-0.17, -0.42, 0.9], mass=100)
                                             position=[0.28, -0.43, 0.9], mass=100)
        # self.box_id = self.create_constraint(shape=BOX, size=[0.1, 0.2, 0.45],
        #                                      position=[-0.17, -0.42, 0.9], mass=100)
        # position=[0.28, -0.43, 0.9], mass=100)
        # self.box_id = self.create_constraint(shape=BOX, size=[0.1, 0.2, 0.45],
        #                                      position=[-0.50, -0.42, 0.9], mass=100)
        # position=[0.28, -0.43, 0.9], mass=100)

        self.world.collision_constraints.append(table_id)
        self.world.toggle_rendering(1)
        self.world.setup_joint_id_to_joint_name()
        self.world.step_simulation_for(0.01)

    def run_simulation(self):
        iteration_count = 0
        while 1:
            if iteration_count < 1:
                iteration_count += 1
                start_state = {}
                goal_state = {}

                start_state["lbr_iiwa_joint_1"] = -2.4823357809267463
                start_state["lbr_iiwa_joint_2"] = 1.4999975516996142
                start_state["lbr_iiwa_joint_3"] = -1.5762726255540713
                start_state["lbr_iiwa_joint_4"] = -0.8666279970481103
                start_state["lbr_iiwa_joint_5"] = 1.5855963769735366
                start_state["lbr_iiwa_joint_6"] = 1.5770985888989753
                start_state["lbr_iiwa_joint_7"] = 1.5704531145724918

                goal_state["lbr_iiwa_joint_1"] = -0.08180533826032865
                goal_state["lbr_iiwa_joint_2"] = 1.5474152457596664
                goal_state["lbr_iiwa_joint_3"] = -1.5873548294514912
                goal_state["lbr_iiwa_joint_4"] = -0.5791571346767671
                goal_state["lbr_iiwa_joint_5"] = 1.5979105177314896
                goal_state["lbr_iiwa_joint_6"] = 1.5857854098720727
                goal_state["lbr_iiwa_joint_7"] = 1.5726221954434347

                duration = 20
                samples = 20
                self.world.reset_joint_states(start_state)
                self.world.step_simulation_for(2)
                check_distance = 0.2
                collision_safe_distance = 0.10
                # self.world.plan_trajectory(goal_state.keys(), goal_state, samples, duration,
                #                      collision_safe_distance=collision_safe_distance,
                #                      collision_check_distance=check_distance)
                self.robot.plan_trajectory(goal_state.keys(),
                                           self.world.get_current_states_for_given_joints(goal_state.keys()),
                                           goal_state, samples, duration,
                                     collision_safe_distance=collision_safe_distance,
                                     collision_check_distance=check_distance,solver_config=None,
                                           callback_funtion=self.world.update_collsion_infos)
                print ("if trajectory has collision: ", \
                self.world.check_for_collision_in_trajectory(self.world.robot.get_trajectory().final, goal_state.keys(),
                                                           collision_safe_distance))
                self.world.execute_trajectory(self.robot.planner.get_trajectory())
                # import sys
                # sys.exit()

if __name__ == '__main__':
    example = PlannerExample()
    example.run_simulation()