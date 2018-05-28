import matplotlib.pyplot as plt
from collections import OrderedDict
from scripts.DB.Mongo_driver import MongoDriver
import numpy as np


# fig, axes = plt.subplots(7, sharex=True)
class Plotter:
    def __init__(self):
        pass

    @classmethod
    def x_y_best_fit_curve(cls, x, y, xlabel, ylabel, title=None, deg=3):
        plt.figure()
        coeffs = np.polyfit(x, y, deg)
        x2 = np.arange(min(x) - 1, max(x) + 1, .01)  # use more points for a smoother plot
        y2 = np.polyval(coeffs, x2)  # Evaluates the polynomial for each x2 value
        fig = plt.figure()
        fig.suptitle(xlabel + ' vs ' + ylabel, fontsize=20)
        plt.xlabel(xlabel, fontsize=18)
        plt.ylabel(ylabel, fontsize=16)
        plt.plot(x2, y2, label="deg=" + str(deg))
        plt.plot(x, y, 'bx')
        plt.show()

    @classmethod
    def plot_xy(self, x, y, xlabel, ylabel, title=None):
        # data = self.db.find({"is_collision_free": True})
        # print data
        # x = []
        # y = []`
        # for d in data:
        #     # print d
        #     x.append(d["planning_time"])
        #     y.append(d["planning_request"]["samples"])

        fig = plt.figure()
        fig.suptitle(xlabel + ' vs ' + ylabel, fontsize=20)
        plt.xlabel(xlabel, fontsize=18)
        plt.ylabel(ylabel, fontsize=16)
        plt.plot(x, y)
        plt.plot(title)
        # plt.plot(x, y)

    @classmethod
    def bar_chart(self, xs, ys, labels, title, c_x_title, c_y_title, width=0.5):

        colors = ['r', 'b', 'g']
        rects = []
        plt.figure()
        for x, y, l, c in zip(xs, ys, labels, colors):
            rects.append(plt.bar(x, y, label=l, color=c, width=width))
            plt.legend(loc='upper left', fontsize=18)
        plt.title(title, fontsize=18)
        plt.xlabel(c_x_title, fontsize=18)
        plt.ylabel(c_y_title, fontsize=18)
        #
        # for r in rects:
        #     self.autolabel(r)

    @classmethod
    def autolabel(self, rects):
        """
        Attach a text label above each bar displaying its height
        """
        for rect in rects:
            height = rect.get_height()
            plt.text(rect.get_x() + rect.get_width() / 2., 1.0 * height,
                     '%.2f' % float(height),
                     ha='center', va='bottom', fontsize=12, fontweight='bold')

    @classmethod
    def bar_chart_side_by_side(self, x, ys, xticks, labels, title, c_x_title, c_y_title, offset=0.2,
                               width=0.3, font_size=18):
        # width = 0.5  # the width of the bars: can also be len(x) sequence
        colors = ['r', 'b', 'g']
        # fig, ax = plt.subplots()
        # fig, ax = plt.subplots()
        fig = plt.figure()
        # for i, ys1 in enumerate(ys):
        rects = []
        for i, (y, l, c) in enumerate(zip(ys, labels, colors)):
            rects.append(plt.bar(x + offset * i, y, label=l, color=c, width=offset))
            plt.legend(fontsize=font_size)
        plt.xticks(x + offset, xticks, fontsize=font_size)
        plt.title(title, fontsize=font_size)
        plt.xlabel(c_x_title, fontsize=font_size)
        plt.ylabel(c_y_title, fontsize=font_size)

        for r in rects:
            self.autolabel(r)


    @classmethod
    def multi_plot_best_fit_curve(self, xs, ys, labels, title, c_x_title, c_y_title, deg=2):
        plt.figure()
        for x, y, l in zip(xs, ys, labels):
            coeffs = np.polyfit(x, y, deg)
            x2 = np.arange(min(x) - 1, max(x) + 1, .01)  # use more points for a smoother plot
            y2 = np.polyval(coeffs, x2)  # Evaluates the polynomial for each x2 value
            # plt.plot(x2, y2, label=l)
            plt.plot(x, y, label=l)
            plt.legend(loc='upper right', fontsize=18)
        plt.title(title, fontsize=18)
        plt.xlabel(c_x_title, fontsize=18)
        plt.ylabel(c_y_title, fontsize=18)

    @classmethod
    def multi_plot_x_y(self, xs, ys, labels, title, c_x_title, c_y_title):
        plt.figure()
        for x, y, l in zip(xs, ys, labels):
            plt.plot(x, y, label=l)
            plt.legend()
            # plt.legend(loc='upper right')
        plt.title(title)
        plt.xlabel(c_x_title)
        plt.ylabel(c_y_title)

    @classmethod
    def multi_plot_1d(self, ys, labels, title, c_x_title, c_y_title):
        plt.figure()
        for y, l in zip(ys, labels):
            plt.plot(y, label=l)
            plt.legend(fontsize=16)
            # plt.legend(loc='upper right')
        plt.title(title, fontsize=20)
        plt.xlabel(c_x_title, fontsize=20)
        plt.ylabel(c_y_title, fontsize=20)

    @classmethod
    def hist(cls, xs, ys, labels, title, c_x_title, c_y_title):

        # colors = ['red', 'tan', 'lime']

        for x, y in zip(xs, ys):
            plt.bar(x, y, label=labels)
            plt.legend()
            # plt.xticks(y + 0.5, ['bin ' + str(int(x)) for x in (y + 0.5)])
            # plt.show()


    @classmethod
    def show(self):
        plt.show()


    @classmethod
    def multi_plot(self, title, initial, final, c_x_title, c_y_title):
        samples = len(final[0])
        x = [i for i in range(samples)]
        num_joints = len(final)
        # Two subplots, the axes array is 1-d
        # global fig, axes
        # fig, axes = plt.subplots(8, sharex=True)
        fig, axes = plt.subplots(num_joints, sharex=True)
        fig.text(0.5, 0.01, c_x_title, ha='center', fontsize=14)
        fig.text(0.01, 0.5, c_y_title, va='center', rotation='vertical', fontsize=14)
        fig.tight_layout(pad=0.2)
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
            ax.set_title(title[i], fontsize=14)
            # ax.autoscale(enable=True, axis='both', tight=False)
            ax.locator_params(nbins=3, axis='y')

        # plt.legend(sub, ['Line Up', 'Line Down'], loc="upper left", bbox_to_anchor=[0, 1],
        #            ncol=2, shadow=True, title="Legend", fancybox=True)
        # plt.legend(["Initial", "Final)"], loc=2, fontsize=14)
        plt.legend(["Initial", "Final)"], loc=9, bbox_to_anchor=(0.9, -0.06), ncol=2, fontsize=14)
        # plt.legend(loc=9, bbox_to_anchor=(0.9, -0.06), ncol=2, fontsize=14)
        # plt.legend(loc="upper left", bbox_to_anchor=[-1, 1],
        #                       ncol=2, shadow=True, title="Legend", fancybox=True)
        fig.suptitle("Trajetory Optimization: Initial vs Final trajectory", fontsize=14)

        #plt.show()
        # plt.show(block=False)

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

    # plotter.plot_xy()
    # plotter.multi_plot()
    Plotter.bar_chart()