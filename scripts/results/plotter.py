import matplotlib.pyplot as plt
from collections import OrderedDict
from scripts.DB.Mongo_driver import MongoDriver
import numpy as np

class Plotter:
    def __init__(self):
        self.db = MongoDriver("trajectory_planner")

    @classmethod
    def plot_xy(self, x, y, xlabel, ylabel, title=None):
        # data = self.db.find({"is_collision_free": True})
        # print data
        # x = []
        # y = []
        # for d in data:
        #     # print d
        #     x.append(d["planning_time"])
        #     y.append(d["planning_request"]["samples"])

        fig = plt.figure()
        fig.suptitle(xlabel + ' vs ' + ylabel, fontsize=20)
        plt.xlabel(xlabel, fontsize=18)
        plt.ylabel(ylabel, fontsize=16)
        plt.plot(x, y, 'bx')
        # plt.plot(x, y)
        plt.show()

    def multi_plot(self, title, initial, final, c_x_title, c_y_title):
        samples = len(final[0])
        x = [i for i in range(samples)]
        num_joints = len(final)
        # Two subplots, the axes array is 1-d
        fig, axes = plt.subplots(num_joints, sharex=True)
        fig.text(0.5, 0.01, c_x_title, ha='center')
        fig.text(0.01, 0.5, c_y_title, va='center', rotation='vertical')
        fig.tight_layout(pad=0.4)
        # Use the pyplot interface to change just one subplot...
        # plt.sca(axes[1])
        # plt.yticks(range(5), [1, 2, 3, 4, 5], color='red')
        sub = []
        sub1 = []
        # for i in range(num_joints):
        for i, ax in enumerate(axes):
            sub.append(ax.plot(x, initial[i], 'b', label='Initial'))
            sub1.append(ax.plot(x, final[i], 'g', label='Final'))
            # ax.plot(x, data[i])
            # plt.plot(x, data[i], 'r')  # plotting t, a separately
            ax.set_title(title[i])
            # ax.autoscale(enable=True, axis='both', tight=False)
            ax.locator_params(nbins=5, axis='y')

        # plt.legend(sub, ['Line Up', 'Line Down'], loc="upper left", bbox_to_anchor=[0, 1],
        #            ncol=2, shadow=True, title="Legend", fancybox=True)
        # plt.legend(["Initial", "Final)"], loc=2, fontsize="small")
        plt.legend(loc=9, bbox_to_anchor=(0.9, -0.06), ncol=2)
        # plt.legend(loc="upper left", bbox_to_anchor=[-1, 1],
        #                       ncol=2, shadow=True, title="Legend", fancybox=True)
        fig.suptitle("Trajetory Optimization: Initial vs Final trajectory", fontsize=14)

        #plt.show()
        plt.show(block=False)

        # for index, trajectory in enumerate(start_state):
            # if (index == 0 or index == len(self.trajectories) - 1):
        # count = 0
        # num_joints = len(start_state)
        # for joint_name, traj in start_state.items():
        #     # plt.plot(traj, label=joint_name, marker='x')
        #     count += 1

            # plt.subplot(7, 1, count)
            # if index == 0:
            #     label = "initial"
            # elif index == 6:
            #     label = "final"
            # plt.plot(traj, label=label, marker='x')

            # ax = plt.subplot(num_joints, 1, count)
            # ax.plot(traj, label=joint_name, marker='x')

            # plt.title('A tale of 2 subplots')
            # plt.ylabel('Damped oscillation')

        # plt.scatter(x, y, c='b', marker='x', label='1')
        # plt.scatter(x, y, c='r', marker='s', label='-1')
        # plt.legend(loc='upper left')
        # plt.show()
        # plt.show(block=False)


if __name__ == '__main__':
    plotter = Plotter()
    # plotter.plot_xy()
    plotter.multi_plot()