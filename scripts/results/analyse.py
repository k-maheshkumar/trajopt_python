from scripts.results.plotter import Plotter as plotter
from scripts.DB.Mongo_driver import MongoDriver
from collections import OrderedDict
from scripts.utils.dict import DefaultOrderedDict
from matplotlib import pyplot as plt
from collections import OrderedDict
import numpy as np


class Analyzer:
    def __init__(self):
        # self.db = MongoDriver("Trajectory_planner_results")
        self.db = MongoDriver("Trajectory_planner_evaluation")

        # self.kuka_results()
        # self.donbot_arm_results()
        # self.donbot_full_results()
        # self.donbot_random_states()
        # self.plot_adatability()
        # results = list(self.db.find({"type": "old_vs_new_solver", "sub_type": "kuka_new_solver"}))
        # results += list(self.db.find({"type": "old_vs_new_solver", "sub_type": "kuka_old_solver"}))
        results = list(self.db.find({"type": "old_vs_new_solver", "sub_type": "donbot_full_new_solver"}))
        results += list(self.db.find({"type": "old_vs_new_solver", "sub_type": "donbot_full_old_solver"}))
        self.plot_old_vs_new_solver(results, "donbot_full")
        plotter.show()

    def plot_old_vs_new_solver(self, results, prefix):

        old_solver_solved = []
        old_solver_unsolved = []

        new_solver_solved = []
        new_solver_unsolved = []

        solved_avg_qp_iterations = []
        solved_avg_sqp_iterations = []
        solved_avg_time = []

        unsolved_avg_qp_iterations = []
        unsolved_avg_sqp_iterations = []
        unsolved_avg_time = []

        for res in results:
            if res["sub_type"] == prefix + "_old_solver" and res["is_collision_free"]:
                old_solver_solved.append(res)
                solved_avg_qp_iterations.append(res["num_qp_iterations"])
                solved_avg_sqp_iterations.append(res["num_sqp_iterations"])
                solved_avg_time.append(res["planning_time"])

            elif res["sub_type"] == prefix + "_new_solver" and res["is_collision_free"]:
                new_solver_solved.append(res)
                unsolved_avg_qp_iterations.append(res["num_qp_iterations"])
                unsolved_avg_sqp_iterations.append(res["num_sqp_iterations"])
                unsolved_avg_time.append(res["planning_time"])
            elif res["sub_type"] == prefix + "_old_solver" and not res["is_collision_free"]:
                old_solver_unsolved.append(res)
            elif res["sub_type"] == prefix + "_new_solver" and not res["is_collision_free"]:
                new_solver_unsolved.append(res)

        solved_avg_qp_iterations = sum(solved_avg_qp_iterations) / (float(len(solved_avg_qp_iterations)) + 1e-4)
        solved_avg_sqp_iterations = sum(solved_avg_sqp_iterations) / (float(len(solved_avg_sqp_iterations)) + 1e-4)
        solved_avg_time = sum(solved_avg_time) / float(len(solved_avg_time) + 1e-4)

        unsolved_avg_qp_iterations = sum(unsolved_avg_qp_iterations) / (float(len(unsolved_avg_qp_iterations)) + 1e-4)
        unsolved_avg_sqp_iterations = sum(unsolved_avg_sqp_iterations) / (float(len(unsolved_avg_sqp_iterations)) + 1e-4)
        unsolved_avg_time = sum(unsolved_avg_time) / (float(len(unsolved_avg_time)) + 1e-4)

        solved_percentage = (len(old_solver_solved) / float(len(old_solver_solved) + len(old_solver_unsolved))) * 100
        unsolved_percentage = (len(new_solver_solved) / float(len(new_solver_solved) + len(new_solver_unsolved))) * 100

        print "len new solved", len(new_solver_solved)
        print "len new unsolved", len(new_solver_unsolved)

        print "len old solved", len(old_solver_solved)
        print "len old unsolved", len(old_solver_unsolved)

        xticks = ['Old vs new \n sovler percentage', 'Number of QP \n iterations', 'Number of SQP \n  iterations',
                  'Average \n solving time']
        x = np.arange(len(xticks))

        # x = [i for i in range(len(xticks))]
        ys = [[solved_percentage, solved_avg_qp_iterations, solved_avg_sqp_iterations, solved_avg_time],
              [unsolved_percentage, unsolved_avg_qp_iterations, unsolved_avg_sqp_iterations, unsolved_avg_time]
              ]
        labels = ["Case: Old solver", "Case: New solver"]

        plotter.bar_chart_side_by_side(x, ys, xticks, labels, prefix + ": Old vs new sovler Trajectory problems",
                                       "", "")


    def plot_adatability(self):
        result = list(self.db.find({"solver_config.trust_region_size": 30}))
        self.plot_avg_planning_time_vs_samples(result)
        results = list(self.db.find({"type": "kuka_only_random_trust_region"}))
        self.plot_avg_planning_time_vs_trust_region(results)
        results = list(self.db.find({"type": "kuka_only_penalty_1_vs_2", "solver_config.penalty_norm": 2}))
        results += list(self.db.find({"type": "kuka_only_penalty_1_vs_2", "solver_config.penalty_norm": 1}))
        self.plot_avg_planning_time_vs_penalty_1_and_2(results)
        results = list(self.db.find({"type": "kuka_only_random_trust_region"}))
        self.iterations_vs_trust_region(results)

    def donbot_full_results(self):
        results = list(self.db.find({"type": "donbot_fullbody_random_state_and_obstacles"}))
        self.plot_reliability(results)
        results = list(self.db.find({"type": "donbot_full_consistency1"}))
        self.plot_consistency(results)

    def donbot_arm_results(self):
        results = list(self.db.find({"type": "donbot_random_state_and_obstacles"}))
        self.plot_reliability(results)
        results = list(self.db.find({"type": "donbot_random_state_and_obstacles"}))
        self.plot_reliability(results)

        results = list(self.db.find({"type": "donbot_arm_consistency1"}))
        self.plot_consistency(results)

    def kuka_results(self):
        # results = list(self.db.find({"solver_config.trust_region_size": 30}))
        # self.plot_avg_planning_time_vs_samples(results)
        # results = list(self.db.find({"type": "kuka_only_random_trust_region"}))
        # self.plot_avg_planning_time_vs_trust_region(results)
        # results = list(self.db.find({"type": "kuka_only_random_trust_region"}))
        # self.iterations_vs_trust_region(results)
        # results = list(self.db.find({"type": "kuka_only_penalty_1_vs_2", "solver_config.penalty_norm": 2}))
        # results += list(self.db.find({"type": "kuka_only_penalty_1_vs_2", "solver_config.penalty_norm": 1}))
        # self.plot_avg_planning_time_vs_penalty_1_and_2(results)

        results = list(self.db.find({"type": "kuka_random_state_and_obstacles"}))
        self.plot_reliability(results)

        results = list(self.db.find({"type": "kuka_consistency"}))
        self.plot_consistency(results)

    def plot_consistency(self, results):

        solved = []
        unsolved = []
        solved_avg_qp_iterations = []
        solved_avg_sqp_iterations = []
        solved_avg_time = []

        unsolved_avg_qp_iterations = []
        unsolved_avg_sqp_iterations = []
        unsolved_avg_time = []

        initial_traj = []
        final_traj = []
        group = results[0]["planning_request"]["group"]

        for res in results:
            if res["is_collision_free"]:
                solved.append(res)
                solved_avg_qp_iterations.append(res["num_qp_iterations"])
                solved_avg_sqp_iterations.append(res["num_sqp_iterations"])
                solved_avg_time.append(res["planning_time"])
                initial_traj.append(res["initial_trajectory"])
                final_traj.append(res["final_trajectory"])
            else:
                unsolved.append(res)
                unsolved_avg_qp_iterations.append(res["num_qp_iterations"])
                unsolved_avg_sqp_iterations.append(res["num_sqp_iterations"])
                unsolved_avg_time.append(res["planning_time"])

        ys = [solved_avg_qp_iterations, solved_avg_sqp_iterations, solved_avg_time]
        labels = ['Number of QP iterations', 'Number of SQP iterations', 'solving time']

        xticks = ['Number of QP \n iterations', 'Number of SQP \n  iterations', 'Average \n solving time']
        x = np.arange(len(xticks))
        labels = ["Case: Problem Solved", "Case: Problem Unsolved"]

        plotter.multi_plot_1d(ys, labels, "Consistent results of Trajectory Solver", "Trails", "Time (S)")

        for i, j in zip(initial_traj, final_traj):
            plotter.multi_plot(group, i, j, "Samples", "Joint angles $\\Theta$")

    def plot_reliability(self, results):

        solved = []
        unsolved = []
        solved_avg_qp_iterations = []
        solved_avg_sqp_iterations = []
        solved_avg_time = []

        unsolved_avg_qp_iterations = []
        unsolved_avg_sqp_iterations = []
        unsolved_avg_time = []

        for res in results:
            if res["is_collision_free"]:
                solved.append(res)
                solved_avg_qp_iterations.append(res["num_qp_iterations"])
                solved_avg_sqp_iterations.append(res["num_sqp_iterations"])
                solved_avg_time.append(res["planning_time"])
            else:
                unsolved.append(res)
                unsolved_avg_qp_iterations.append(res["num_qp_iterations"])
                unsolved_avg_sqp_iterations.append(res["num_sqp_iterations"])
                unsolved_avg_time.append(res["planning_time"])

        solved_avg_qp_iterations = sum(solved_avg_qp_iterations) / (float(len(solved_avg_qp_iterations)) + 1e-4)
        solved_avg_sqp_iterations = sum(solved_avg_sqp_iterations) / (float(len(solved_avg_sqp_iterations)) + 1e-4)
        solved_avg_time = sum(solved_avg_time) / len(solved_avg_time)

        unsolved_avg_qp_iterations = sum(unsolved_avg_qp_iterations) / (float(len(unsolved_avg_qp_iterations)) + 1e-4)
        unsolved_avg_sqp_iterations = sum(unsolved_avg_sqp_iterations) / (float(len(unsolved_avg_sqp_iterations)) + 1e-4)
        unsolved_avg_time = sum(unsolved_avg_time) / (float(len(unsolved_avg_time)) + 1e-4)

        solved_percentage = (len(solved) / float(len(results))) * 100
        unsolved_percentage = (len(unsolved) / float(len(results))) * 100

        xticks = ['Solved vs Unsolved \n percentage', 'Number of QP \n iterations', 'Number of SQP \n  iterations',
                   'Average \n solving time']
        x = np.arange(len(xticks))

        # x = [i for i in range(len(xticks))]
        ys = [[solved_percentage, solved_avg_qp_iterations, solved_avg_sqp_iterations, solved_avg_time],
             [unsolved_percentage, unsolved_avg_qp_iterations, unsolved_avg_sqp_iterations, unsolved_avg_time]
              ]
        labels = ["Case: Problem Solved", "Case: Problem Unsolved"]

        plotter.bar_chart_side_by_side(x, ys, xticks, labels, "Solved vs Unsolved Trajectory problems",
                                       "", "")


    def plot_avg_planning_time_vs_samples(self, results):
        # result = list(self.db.find({"solver_config.trust_region_size": 30}))
        # print len(result)
        sam_t = DefaultOrderedDict(list)
        sol_t = DefaultOrderedDict(list)
        col_t = DefaultOrderedDict(list)
        prb_t = DefaultOrderedDict(list)
        cost = DefaultOrderedDict(list)
        for res in results:
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

        ys = [avg_solving_time, avg_collision_check_time, avg_prob_model_time]
        xs = [samples] * len(ys)

        labels = ["solving_time", "collision_check_time", "prob_model_time"]
        # plotter.multi_plot_best_fit_curve(xs, ys, labels, "Time vs Number of samples", "Number of samples", "Time (S)",
        plotter.bar_chart(xs, ys, labels, "Time taken vs Number of samples", "Number of samples", "Average Time (S)")

    def plot_avg_planning_time_vs_trust_region(self, results):
        # results = list(self.db.find({"type": "kuka_only_random_trust_region"}))
        # result = list(self.db.find({"type": "donbot_arm_only_random_trust_region"}))
        # print len(result)
        trust_region = DefaultOrderedDict(list)
        plan_t = DefaultOrderedDict(list)
        sol_t = DefaultOrderedDict(list)
        col_t = DefaultOrderedDict(list)
        prb_t = DefaultOrderedDict(list)
        for res in results:
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
        plotter.multi_plot_best_fit_curve(xs, ys, labels, "Average time taken vs Trust region size", "Trust region size", "Average Time (S)",
                                          deg=2)

    def iterations_vs_trust_region(self, results):
        trust_region = DefaultOrderedDict(list)
        qp_iters = DefaultOrderedDict(list)
        sqp_iters = DefaultOrderedDict(list)
        for res in results:
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
        plotter.multi_plot_best_fit_curve(xs, ys, labels, "Number of SQP and QP iterations vs Trust region", "Trust region", "Number of SQP and QP iterations",
                                          deg=2)

    def plot_avg_planning_time_vs_penalty_1_and_2(self, results):
        penalty_norm = DefaultOrderedDict(list)
        sol_t = DefaultOrderedDict(list)
        col_t = DefaultOrderedDict(list)
        prb_t = DefaultOrderedDict(list)
        penalty_norm1 = DefaultOrderedDict(list)
        sol_t1 = DefaultOrderedDict(list)
        col_t1 = DefaultOrderedDict(list)
        prb_t1 = DefaultOrderedDict(list)
        for res in results:
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

        labels = ["$l_1$ penalty: planning_time", "$l_1$ penalty: solving_time",
                  "$l_1$ penalty: collision_check_time", "$l_1$ penalty: prob_model_time",
                  "$l_2$ penalty 2: planning_time", "$l_2$ penalty: solving_time",
                  "$l_2$ penalty: collision_check_time", "$l_2$ penalty: prob_model_time"]
        plotter.multi_plot_best_fit_curve(xs, ys, labels, "$l_1$ Penalty vs $l_2$ penalty", "Number of samples", "Average time (S)",
                                          deg=2)

    def get_rounded_off_list(self, data, decimal=3):
        if data is list:
            return [round(float(d), decimal) for d in data]
        else:
            return round(float(data), decimal)




if __name__ == '__main__':

    res = Analyzer()

