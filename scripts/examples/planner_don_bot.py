import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), "../../"))

from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
import pybullet as p
from random import randint
from collections import OrderedDict
from random import randrange, uniform, randint

home = os.path.expanduser('~')


class PlannerExample:
    def __init__(self):

        shelf_file = home + "/catkin_ws/src/iai_shelf_description/urdf/shelf.urdf"

        config = {
            "use_gui": True,
            # "verbose": "INFO",
            "log_file": True,
            # "save_problem": True,
            # "plot_trajectory": True,
            "db_name": "Trajectory_planner_evaluation",
            "robot_config": "robot_config_don_bot.yaml"
        }

        self.planner = TrajectoryOptimizationPlanner(**config)
        self.planner.world.toggle_rendering(0)
        self.planner.world.set_gravity(0, 0, -10)
        plane_id = self.planner.add_constraint_from_urdf("plane", "plane.urdf", position=[0, 0, 0.0])

        self.planner.world.toggle_rendering(0)
        shelf_id = self.planner.add_constraint_from_urdf("shelf", urdf_file=shelf_file, position=[-0.45, 0, 0.0],
                                                         orientation=p.getQuaternionFromEuler([0, 0, 1.57]))

        shelf_item_prefix = home + "/catkin_ws/src/shelf_item_descriptions/urdf/"
        salt_urdf = shelf_item_prefix + "salt.urdf"
        gel_urdf = shelf_item_prefix + "duschGel.urdf"
        gel_id = OrderedDict()

        y, z = 0.3, 1.0
        offset = -0.58
        # offset = uniform(-0.7, -0.58)
        zs = [0.2, 0.6, 1]
        # z = 1.4
        # z = 1
        obj_at_shelf = randint(1, 4)
        obj_at_shelf = 3
        for x in range(obj_at_shelf):
            y = uniform(-0.2, 0.2)
            # z = uniform(0.3, 1.5)
            z = randint(0, 2)
            gel_id[x] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=gel_urdf,
                                                              position=[offset + 0.1 * x, y, zs[z]])
            gel_id[x + 4] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=gel_urdf,
                                                                  position=[offset + 0.1 * x, y - 0.38, zs[z]])
        # y, z = 0.1, 0.719
        # offset = 0.38
        # # x = uniform(-0.5, 0.5)
        # obj_at_bot = randint(1, 4)
        # # obj_at_bot = 3
        # zs = [0.2, 0.6, 1]
        # # z = 2
        # lotion_urdf = shelf_item_prefix + "bodyLotion.urdf"
        # for x in range(obj_at_bot):
        #     y = uniform(-0.2, 0.2)
        #     # z = randint(0, 2)
        #     gel_id[x] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=lotion_urdf,
        #                                                       # position=[offset + 0.1 * x, y, zs[z]])
        #                                                       position=[offset + 0.1 * x, y, z])
        #     gel_id[x + 4] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=lotion_urdf,
        #                                                        # position=[offset + 0.1 * x, y-0.38, zs[z]])
        #                                                        position=[offset + 0.1 * x, y-0.38, z])
        # # # gel_id = OrderedDict()
        # y, z = -0.3, 0.62
        # offset = -0.59
        # for x in range(2):
        #     gel_id[x] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=gel_urdf,
        #                                                       position=[offset + 0.1 * x, y, z])
        # for x in range(2):
        #     gel_id[x] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=salt_urdf,
        #                                                       position=[offset + 0.1 * x, 0.3, z])
        # for x in range(1):
        #     gel_id[x + 4] = self.planner.add_constraint_from_urdf("gel" + str(x + 4), urdf_file=gel_urdf,
        #                                                           position=[offset + 0.1 * x, y - 0.14, z])
        # lotion_urdf = shelf_item_prefix + "bodyLotion.urdf"
        # lotion_id = OrderedDict()
        # y, z = -0.4, 1
        # offset = -0.59
        # for x in range(1):
        #     lotion_id[x] = self.planner.add_constraint_from_urdf("lotion" + str(x), urdf_file=lotion_urdf,
        #                                                          position=[offset + 0.1 * x, y, z])

        self.planner.world.toggle_rendering(1)

    def run(self):
        start = randint(1, 5)
        end = randint(6, 10)

        start_state = "aloc" + str(start)
        goal_state = "aloc" + str(end)
        group = "ur5_arm1"

        # start_state = "floc" + str(start)
        # goal_state = "floc" + str(end)
        # group = "full_body"

        self.planner.reset_robot_to(start_state, group)

        duration = 10
        samples = 20
        collision_check_distance = 0.15
        collision_safe_distance = 0.1

        ignore_goal_states = ["odom_x_joint", "odom_y_joint", "odom_z_joint", "gripper_base_gripper_left_joint",
                              "gripper_joint"]

        # start_state = OrderedDict(
        #     [('ur5_shoulder_pan_joint', 1.6534695625305176), ('ur5_shoulder_lift_joint', -2.1164417266845703),
        #      ('ur5_elbow_joint', -2.2487189769744873), ('ur5_wrist_1_joint', 0.5952491760253906),
        #      ('ur5_wrist_2_joint', 1.984163761138916), ('ur5_wrist_3_joint', -1.6534700393676758),
        #      ('gripper_base_gripper_left_joint', -0.0027000010013580322), ('gripper_joint', 0.006500000134110451)])

        _, status, trajectory = self.planner.get_trajectory(samples=samples, duration=duration,
                                                            group=group, goal_state=goal_state,
                                                            # start_state=start_state,
                                                            # group=group1, goal_state=goal_state1,
                                                            collision_safe_distance=collision_safe_distance,
                                                            collision_check_distance=collision_check_distance,
                                                            ignore_goal_states=ignore_goal_states
                                                            )
        print("is trajectory free from collision: ", status)

        if status:
            self.planner.execute_trajectory()

    def manual_control(self):
        start_state = "below_shelf"
        group = "full_body"
        start = randint(1, 4)
        end = randint(5, 8)

        start_state = "aloc" + str(start)
        goal_state = "aloc" + str(end)
        group = "ur5_arm"

        self.planner.reset_robot_to(start_state, group)

        group = self.planner.get_group_names(group)
        self.planner.world.manual_control(self.planner.robot.id, group, file_name="./donbot_state.yaml", use_current_state=True)


def main():
    example = PlannerExample()
    example.run()
    # example.manual_control()


if __name__ == '__main__':
    main()
