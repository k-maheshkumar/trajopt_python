import matplotlib.pyplot as plt
from collections import OrderedDict
from scripts.DB.Mongo_driver import MongoDriver
import numpy as np

class Plotter:
    def __init__(self):
        self.db = MongoDriver("trajectory_planner")

    def plot_xy(self, data=None):
        data = self.db.find({"is_collision_free": True})
        print data
        x = []
        y = []
        for d in data:
            # print d
            x.append(d["planning_time"])
            y.append(d["planning_request"]["samples"])
        print x, y
        plt.plot(x, y, 'ro')
        plt.show()

    def multi_plot(self, title, data, c_x_title, c_y_title):

        # data = [[-2.4823299997268826, 1.499999999820583, -1.5762100000921369, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-2.1563588369825495, 1.2609677221506188, -1.735981551143503, -1.1330610052452825, 1.5271507220692668,
        #          1.6720202595866538, 1.6868032425694888, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-1.8303876738951297, 1.0219354413125354, -1.8957531046904548, -1.399492008023908, 1.4687014470015594,
        #          1.7669405056438807, 1.8031564879486575, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-1.5044166633289933, 0.7829030806244154, -2.055524731163904, -1.665923127582652, 1.4102521488942557,
        #          1.8618607424715186, 1.9195097359487259, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-1.1784456832955212, 0.5438706889788141, -2.215296384185213, -1.9323542626380505, 1.3518028489710545,
        #          1.9567809515981494, 2.035862986698262, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-1.0256953477880812, 0.524042649486253, -2.2271466089000898, -1.8987962908540794, 1.3560460318650875,
        #          1.9561950682556557, 2.0029889557100917, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-0.8729450141582424, 0.5042146126412577, -2.238996833169753, -1.8652382789461464, 1.3602892189465987,
        #          1.9556091188012508, 1.9701149283399821, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-0.7201946809812545, 0.4843865789922306, -2.2508470564333964, -1.8316802314707024, 1.3645324111450947,
        #          1.9550231076565063, 1.9372409047802193, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-0.5674443503804878, 0.46455854954155773, -2.2626972786218866, -1.7981221566265284, 1.3687756083940337,
        #          1.9544370309171355, 1.904366884692048, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-0.4146940227316527, 0.44473052384183465, -2.2745475000319093, -1.7645640645779028, 1.3730188107721268,
        #          1.9538508872766265, 1.8714928685358216, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-0.26194380608411705, 0.42490245860845793, -2.286397776845338, -1.7310060124205362, 1.377262012119762,
        #          1.9532646670443938, 1.8386188570289876, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-0.10919688724194732, 0.40507317344038984, -2.2982496469338916, -1.6974496926820093, 1.381504891469545,
        #          1.9526781151573385, 1.8057448530637332, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [0.09169478343586296, 0.5994021007434134, -2.1495428089741337, -1.4371988579643982, 1.4327143548396932,
        #          1.9143336245008051, 1.772871112080103, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [0.27361823415044756, 0.7999697229431486, -1.9991115728931, -1.17537386943406, 1.4841402000585069,
        #          1.8717626144407773, 1.739999041087984, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [0.22461833237785545, 0.9287493151385059, -1.9285089930759929, -1.0684722902613453, 1.5029460719312044,
        #          1.8241005500036662, 1.7121025295836905, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [0.1756189087976497, 1.0575288696489897, -1.8579064284619955, -0.9615706209929933, 1.5217519548736513,
        #          1.776438466432915, 1.684206017052633, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [0.11126168157186855, 1.1800016549425958, -1.7902673186699454, -0.8659679669477814, 1.5407914612964042,
        #          1.728776370172664, 1.656309508173266, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [0.04690445453602915, 1.3024744398333192, -1.7226282105756485, -0.7703653119652306, 1.559830971190371,
        #          1.681114258922838, 1.628413002512713, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-0.017452772611564638, 1.4249472223277886, -1.6549891041977955, -0.6747626562654186,
        #          1.5788704841385555, 1.6334521350721445, 1.6005164997243382, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208],
        #         [-0.0818099999664728, 1.547419999902837, -1.587350000032182, -0.5791600000740404, 1.597909999981596,
        #          1.5857900000230438, 1.5726200000110697, -0.8666300001906163, 1.585599999954932,
        #          1.5771000000612572, 1.5704500000708208]]

        # data = np.asarray(data).tolist()
        print len(data[0])
        samples = len(data[0])
        x = [i for i in range(samples)]
        num_joints = len(data)
        print "num_joints", num_joints
        # num_joints = 7
        # Two subplots, the axes array is 1-d
        fig, axes = plt.subplots(num_joints, sharex=True)
        fig.text(0.5, 0.01, c_x_title, ha='center')
        fig.text(0.01, 0.5, c_y_title, va='center', rotation='vertical')
        fig.tight_layout(pad=0.4)
        # Use the pyplot interface to change just one subplot...
        # plt.sca(axes[1])
        # plt.yticks(range(5), [1, 2, 3, 4, 5], color='red')

        # for i in range(num_joints):
        for i, ax in enumerate(axes):
            # print title[i]
            # print len(axes)
            # print len(data[i])
            ax.plot(x, data[i])
            ax.set_title(title[i])
            # ax.autoscale(enable=True, axis='both', tight=False)
            ax.locator_params(nbins=5, axis='y')

        plt.show()

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