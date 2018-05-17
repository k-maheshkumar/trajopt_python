import matplotlib.pyplot as plt

class Plotter:
    def __init__(self):
        pass

    @classmethod
    def plot_1d(self, x, block=True):
        plt.plot(x, 'ro')
        plt.show(block=block)

    @classmethod
    def plot_xy(self, x, y, block=True):
        plt.plot(x, y, 'ro')
        plt.show(block=block)

    @classmethod
    def multi_plot(self, title, initial, final, c_x_title, c_y_title, block=True):
        samples = len(final[0])
        x = [i for i in range(samples)]
        num_joints = len(final)
        # Two subplots, the axes array is 1-d
        fig, axes = plt.subplots(num_joints, sharex=True)
        fig.text(0.5, 0.01, c_x_title, ha='center')
        fig.text(0.01, 0.5, c_y_title, va='center', rotation='vertical')
        fig.tight_layout(pad=0.4)
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

        plt.ion()
        plt.show(block=block)
