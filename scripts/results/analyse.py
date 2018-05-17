from scripts.results.plotter import Plotter as plotter
from scripts.DB.Mongo_driver import MongoDriver
from collections import OrderedDict
from scripts.utils.dict import DefaultOrderedDict
import csv
import sys
from matplotlib import pyplot as plt
from collections import OrderedDict


class Analyzer:
    def __init__(self):
        print "hello"
        # self.db = MongoDriver("Trajectory_planner_results")
        self.db = MongoDriver("Trajectory_planner_evaluation")
        result = self.db.find({})
        # print result[0]
        # x = []
        # y = []
        # print result
        # for res in result:
        #     print res
        #     # if res["is_collision_free"]:
        #     #     print "cost improvement", res["prob_costs"]
        #         # print "cost improvement---", res["final_cost"] / (res["initial_cost"] + 1e-3)
        #         # print "cost improvement", res["cost improvement"]
        #
        #     # print d
        #     x.append(res["planning_request"]["samples"])
        #     y.append(res["planning_time"])
        #     # y.append(res["cost_improvement"])
        #
        # plotter.plot_xy(x, y, "samples", "planning_time")
        self.plot_avg_planning_time_vs_samples()

    def plot_avg_planning_time_vs_samples(self):
        result = list(self.db.find({"solver_config.trust_region_size": 30}))
        # print len(result)
        data = DefaultOrderedDict(list)
        data1 = DefaultOrderedDict(list)
        data2 = DefaultOrderedDict(list)
        data3 = DefaultOrderedDict(list)
        for res in result:
            sam = res["planning_request"]["samples"]
            data[sam].append(res["planning_time"])
            data1[sam].append(res["solving_time"])
            data2[sam].append(res["collision_check_time"])
            data3[sam].append(res["prob_model_time"])
        samples = []
        planning_time = []
        solving_time = []
        collision_check_time = []
        prob_model_time = []
        data = OrderedDict(sorted(data.items()))
        for k in data:
            print k, len(data[k])
            samples.append(k)
            planning_time.append(sum(data[k]) / len(data[k]))
            solving_time.append(sum(data1[k]) / len(data1[k]))
            collision_check_time.append(sum(data2[k]) / len(data2[k]))
            prob_model_time.append(sum(data3[k]) / len(data3[k]))

            # if k == 18:
            #     print len(v)
        # plotter.plot_xy(x, y, "samples", "planning_time")
        # plotter.x_y_best_fit_curve(x, y, "samples", "planning_time", deg=5)
        # plotter.x_y_best_fit_curve(samples, prob_model_time, "samples", "prob_model_time", deg=5)
        ys = [planning_time, solving_time, collision_check_time, prob_model_time]
        xs = [samples] * len(ys)
        print sorted(samples)
        labels = ["planning_time", "solving_time", "collision_check_time", "prob_model_time"]
        plotter.multi_plot_best_fit_curve(xs, ys, labels, "Time vs Number of samples", "Number of samples", "Time (S)",
                                          deg=8)
            # print data[d]
        # print data
        # x = []
        # y = []
        # print result
        # for res in result:
        #     print res
        #     # if res["is_collision_free"]:
        #     #     print "cost improvement", res["prob_costs"]
        #         # print "cost improvement---", res["final_cost"] / (res["initial_cost"] + 1e-3)
        #         # print "cost improvement", res["cost improvement"]
        #
        #     # print d
        #     x.append(res["planning_request"]["samples"])
        #     y.append(res["planning_time"])
        #     # y.append(res["cost_improvement"])
        #
        # plotter.plot_xy(x, y, "samples", "planning_time")


    def temp(self):
        data = []

        # planning_request["samples"] = samples
        # planning_request["duration"] = duration
        # planning_request["group"] = group
        # planning_request["start_state"] = current_robot_state
        # planning_request["goal_state"] = goal_state
        # planning_request["no of links"] = len(self.world.robot_info["joint_infos"])
        # planning_request["collision_safe_distance"] = collision_safe_distance
        # planning_request["collision_check_distance"] = collision_check_distance
        # result = OrderedDict()
        # result["num_iterations"] = self.robot.planner.sqp_solver.num_iterations
        # result["initial_cost"] = self.robot.planner.sqp_solver.initial_cost
        # result["final_cost"] = self.robot.planner.sqp_solver.final_cost
        # result["cost_improvement"] = improve
        # result["collision_check_time"] = self.world.collision_check_time
        # result["solving_time"] = self.robot.planner.sqp_solver.solving_time
        # result["prob_model_time"] = self.robot.planner.prob_model_time
        # result["planning_time"] = total
        # result["is_collision_free"] = is_collision_free
        # result["planning_request"] = planning_request
        # result["trajectory"] = self.robot.planner.trajectory.final.tolist()
        # result["solver_config"] = self.robot.planner.sqp_solver.solver_config
        # with open('./result.csv', 'wb') as f:
        # d = DefaultOrderedDict(OrderedDict)
        # for res in result:
        #     if res["is_collision_free"] and res["cost_improvement"] != 0:
        #         d = DefaultOrderedDict(OrderedDict)
        #         samples = res["planning_request"]["samples"]
        #         d[samples]["samples"] = res["planning_request"]["samples"]
        #         d[samples]["num_iterations"] = res["num_iterations"]
        #         d[samples]["planning_time"] = res["planning_time"]
        #         d[samples]["cost_improvement"] = res["cost_improvement"]
        #         d[samples]["collision_check_time"] = res["collision_check_time"]
        #         d[samples]["prob_model_time"] = res["prob_model_time"]

                    # data.append(d)

                    # w = csv.DictWriter(f, d.keys())
                    # # w = csv.DictWriter(sys.stderr, d.keys())
                    # # w.writeheader()
                    # # w.writerow(d.keys())
                    # w.writerow(d)
                    # # w.writerows(d.items())

        # data = DefaultOrderedDict(list)
        # for res in result:
        #     if res["is_collision_free"] and res["cost_improvement"] != 0:
        #         d = DefaultOrderedDict(OrderedDict)
        #         samples = res["planning_request"]["samples"]
        #         # d[samples]["samples"] = res["planning_request"]["samples"]
        #         d[samples]["num_iterations"] = res["num_iterations"]
        #         d[samples]["planning_time"] = res["planning_time"]
        #         d[samples]["cost_improvement"] = res["cost_improvement"]
        #         d[samples]["collision_check_time"] = res["collision_check_time"]
        #         d[samples]["prob_model_time"] = res["prob_model_time"]
        #
        #         data[samples].append(d)

        # print data[23]
        # self.num_iterations = DefaultOrderedDict(float)
        # self.cost_improvement = DefaultOrderedDict(float)
        # self.collision_check_time = DefaultOrderedDict(float)
        # self.prob_model_time = DefaultOrderedDict(float)
        # self.planning_time = DefaultOrderedDict(float)
        #
        # self.get_result()

    def get_avg(self):
        for k, v in data.iteritems():
            # print k, v
            for d in v:
                if k == 23:
                    self.num_iterations[k] += v

    def get_result(self):

        for i in range(30):
            result = self.db.find({"planning_request.samples": i})
            count = 0
            for res in result:
                if res["is_collision_free"]:
                    # print res
                    print res["num_iterations"]
                    count += 1
                    self.num_iterations[i] += res["num_iterations"]
                    self.cost_improvement[i] += res["cost_improvement"]
                    self.collision_check_time[i] += res["collision_check_time"]
                    self.prob_model_time[i] += res["prob_model_time"]
                    self.planning_time[i] += res["planning_time"]
                    # print i, res["num_iterations"], count
                if count:
                    self.num_iterations[i] /= count
                    self.cost_improvement[i] /= count
                    self.collision_check_time[i] /= count
                    self.prob_model_time[i] /= count
                    self.planning_time[i] /= count
        print self.num_iterations
        print self.cost_improvement
        print self.collision_check_time
        print self.prob_model_time
        print self.planning_time

        for (k,v), (k1,v1), (k2,v2), (k3,v3) in zip(self.num_iterations.items(), self.planning_time.items(),
                                           self.collision_check_time.items(), self.prob_model_time.items()):
            print v, v2
            x = v / 2
            y = v1
            z = v2 * 1
            k = v3 * 100
            ax = plt.subplot(111)
            w = 0.3
            ax.bar(x - w, y, width=w, color='b', align='center')
            ax.bar(x, z, width=w, color='g', align='center')
            ax.bar(x + w, k, width=w, color='r', align='center')
            # ax.xaxis_date()
            ax.autoscale(tight=True)

        plt.show()


if __name__ == '__main__':

    res = Analyzer()