from scripts.results.plotter import Plotter as plotter
from scripts.DB.Mongo_driver import MongoDriver
from collections import OrderedDict
from scripts.utils.dict import DefaultOrderedDict
from matplotlib import pyplot as plt
from collections import OrderedDict


class Analyzer:
    def __init__(self):
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
        self.plot_avg_planning_time_vs_trust_region()
        self.plot_avg_planning_time_vs_penalty_1_and_2()
        self.iterations_vs_trust_region()
        plotter.show()

    def plot_avg_planning_time_vs_samples(self):
        result = list(self.db.find({"solver_config.trust_region_size": 30}))
        # print len(result)
        sam_t = DefaultOrderedDict(list)
        sol_t = DefaultOrderedDict(list)
        col_t = DefaultOrderedDict(list)
        prb_t = DefaultOrderedDict(list)
        cost = DefaultOrderedDict(list)
        for res in result:
            sam = res["planning_request"]["samples"]
            sam_t[sam].append(res["planning_time"])
            sol_t[sam].append(res["solving_time"])
            col_t[sam].append(res["collision_check_time"])
            prb_t[sam].append(res["prob_model_time"])
            a_cost = res["actual_reductions"]
            imp = a_cost[0] - a_cost[-1]
            imp /= a_cost[0]
            imp *= 100
            cost[sam].append(imp)
            # print sam, imp
        # print cost
        samples = []
        avg_planning_time = []
        avg_solving_time = []
        avg_collision_check_time = []
        avg_prob_model_time = []
        avg_cost = []
        sam_t = OrderedDict(sorted(sam_t.items()))
        for k in sam_t:
            # print k, len(sam_t[k])
            samples.append(k)
            avg_planning_time.append(sum(sam_t[k]) / len(sam_t[k]))
            avg_solving_time.append(sum(sol_t[k]) / len(sol_t[k]))
            avg_collision_check_time.append(sum(col_t[k]) / len(col_t[k]))
            avg_prob_model_time.append(sum(prb_t[k]) / len(prb_t[k]))
            avg_cost.append(sum(cost[k]) / len(cost[k]))
        ys = [avg_planning_time, avg_solving_time, avg_collision_check_time, avg_prob_model_time]
        xs = [samples] * len(ys)
        # print samples
        # print self.get_rounded_off_list(avg_planning_time)
        # print self.get_rounded_off_list(avg_solving_time)
        # print self.get_rounded_off_list(avg_collision_check_time)
        # print self.get_rounded_off_list(avg_prob_model_time, 5)
        # print self.get_rounded_off_list(avg_cost, 3)
        labels = ["planning_time", "solving_time", "collision_check_time", "prob_model_time"]
        plotter.multi_plot_best_fit_curve(xs, ys, labels, "Time vs Number of samples", "Number of samples", "Time (S)",
                                          deg=8)

    def plot_avg_planning_time_vs_trust_region(self):
        result = list(self.db.find({"type": "kuka_only_random_trust_region"}))
        # result = list(self.db.find({"type": "donbot_arm_only_random_trust_region"}))
        # print len(result)
        trust_region = DefaultOrderedDict(list)
        plan_t = DefaultOrderedDict(list)
        sol_t = DefaultOrderedDict(list)
        col_t = DefaultOrderedDict(list)
        prb_t = DefaultOrderedDict(list)
        for res in result:
            trust = res["solver_config"]["trust_region_size"]
            trust_region[trust].append(trust)
            plan_t[trust].append(res["planning_time"])
            sol_t[trust].append(res["solving_time"])
            col_t[trust].append(res["collision_check_time"])
            prb_t[trust].append(res["prob_model_time"])
        avg_planning_time = []
        avg_solving_time = []
        avg_collision_check_time = []
        avg_prob_model_time = []
        trust_region = OrderedDict(sorted(trust_region.items()))
        for k in trust_region:
            avg_planning_time.append(sum(plan_t[k]) / len(plan_t[k]))
            avg_solving_time.append(sum(sol_t[k]) / len(sol_t[k]))
            avg_collision_check_time.append(sum(col_t[k]) / len(col_t[k]))
            avg_prob_model_time.append(sum(prb_t[k]) / len(prb_t[k]))
        ys = [avg_planning_time, avg_solving_time, avg_collision_check_time, avg_prob_model_time]
        xs = [trust_region.keys()] * len(ys)
        labels = ["planning_time", "solving_time", "collision_check_time", "prob_model_time"]
        plotter.multi_plot_best_fit_curve(xs, ys, labels, "Time vs Number of samples", "Time", "Time (S)",
                                          deg=2)

    def iterations_vs_trust_region(self):
        result = list(self.db.find({"type": "kuka_only_random_trust_region"}))
        trust_region = DefaultOrderedDict(list)
        qp_iters = DefaultOrderedDict(list)
        sqp_iters = DefaultOrderedDict(list)
        for res in result:
            trust = res["solver_config"]["trust_region_size"]
            trust_region[trust].append(trust)
            qp_iters[trust].append(res["num_qp_iterations"])
            sqp_iters[trust].append(res["num_sqp_iterations"])
        avg_qp_iters = []
        avg_sqp_iters = []
        trust_region = OrderedDict(sorted(trust_region.items()))
        for k in trust_region:
            avg_qp_iters.append(sum(qp_iters[k]) / len(qp_iters[k]))
            avg_sqp_iters.append(sum(sqp_iters[k]) / len(sqp_iters[k]))
        ys = [avg_qp_iters, avg_sqp_iters]
        xs = [trust_region.keys()] * len(ys)
        labels = ["Number of qp iterations", "Num sqp iterations"]
        plotter.multi_plot_best_fit_curve(xs, ys, labels, "Time vs Number of samples", "Time", "Time (S)",
                                          deg=2)

    def plot_avg_planning_time_vs_penalty_1_and_2(self):
        result = list(self.db.find({"type": "kuka_only_penalty_1_vs_2", "solver_config.penalty_norm": 2}))
        result += list(self.db.find({"type": "kuka_only_penalty_1_vs_2", "solver_config.penalty_norm": 1}))
        penalty_norm = DefaultOrderedDict(list)
        sol_t = DefaultOrderedDict(list)
        col_t = DefaultOrderedDict(list)
        prb_t = DefaultOrderedDict(list)
        penalty_norm1 = DefaultOrderedDict(list)
        sol_t1 = DefaultOrderedDict(list)
        col_t1 = DefaultOrderedDict(list)
        prb_t1 = DefaultOrderedDict(list)
        for res in result:
            p = res["solver_config"]["penalty_norm"]
            if p == 1:
                trust = res["solver_config"]["trust_region_size"]
                penalty_norm[trust].append(res["planning_time"])
                sol_t[trust].append(res["solving_time"])
                col_t[trust].append(res["collision_check_time"])
                prb_t[trust].append(res["prob_model_time"])
            elif p == 2:
                trust = res["solver_config"]["trust_region_size"]
                penalty_norm1[trust].append(res["planning_time"])
                sol_t1[trust].append(res["solving_time"])
                col_t1[trust].append(res["collision_check_time"])
                prb_t1[trust].append(res["prob_model_time"])

        tr = []
        avg_planning_time = []
        avg_solving_time = []
        avg_collision_check_time = []
        avg_prob_model_time = []
        penalty_norm = OrderedDict(sorted(penalty_norm.items()))
        for k in penalty_norm:
            # print k, len(penalty_norm[k])
            tr.append(k)
            avg_planning_time.append(sum(penalty_norm[k]) / len(penalty_norm[k]))
            avg_solving_time.append(sum(sol_t[k]) / len(sol_t[k]))
            avg_collision_check_time.append(sum(col_t[k]) / len(col_t[k]))
            avg_prob_model_time.append(sum(prb_t[k]) / len(prb_t[k]))
        tr1 = []
        avg_planning_time1 = []
        avg_solving_time1 = []
        avg_collision_check_time1 = []
        avg_prob_model_time1 = []
        penalty_norm1 = OrderedDict(sorted(penalty_norm.items()))
        for k in penalty_norm1:
            print k, len(penalty_norm1[k])
            tr1.append(k)
            avg_planning_time1.append(sum(penalty_norm1[k]) / len(penalty_norm1[k]))
            avg_solving_time1.append(sum(sol_t1[k]) / len(sol_t1[k]))
            avg_collision_check_time1.append(sum(col_t1[k]) / len(col_t1[k]))
            avg_prob_model_time1.append(sum(prb_t1[k]) / len(prb_t1[k]))

        ys = [avg_planning_time, avg_solving_time, avg_collision_check_time, avg_prob_model_time,
              avg_planning_time1, avg_solving_time1, avg_collision_check_time1, avg_prob_model_time1]
        xs = [tr, tr1] * (len(ys) / 2)

        labels = ["penalty 1: planning_time", "penalty 1: solving_time",
                  "penalty 1: collision_check_time", "penalty 1: prob_model_time",
                  "penalty 2: planning_time1", "penalty 2: solving_time1",
                  "penalty 2: collision_check_time1", "penalty 2: prob_model_time1"]
        plotter.multi_plot_best_fit_curve(xs, ys, labels, "Time vs Number of samples", "Time", "Time (S)",
                                          deg=2)

    def get_rounded_off_list(self, data, decimal=3):
        return [round(float(d), decimal) for d in data]

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



if __name__ == '__main__':

    res = Analyzer()