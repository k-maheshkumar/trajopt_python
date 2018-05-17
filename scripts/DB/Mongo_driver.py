from pymongo import Connection
from pymongo import MongoClient


class MongoDriver():
    def __init__(self, db_name="Trajectory_planner_results", host='localhost', port=27017):
        self.client = MongoClient(host, port)
        # self.client = Connection()
        self.db_collection = self.client[db_name]
        self.db = self.db_collection.db_name

    def find_one(self, query):
        return self.db.find_one(query)

    def find(self, query):
        return self.db.find(query)

    def insert(self, request):
        # print request
        self.db.insert(request)
        print "saved request into database . . . . . . .  ."

    def say_hello(self):
        print "hello . . ."

    def drop(self):
        self.db.drop()

    def remove_item(self, request):
        self.db.remove(request)


def main():

    # db = MongoDriver("Trajectory_planner_results")
    db = MongoDriver("Trajectory_planner_evaluation")
    # db = MongoDriver()
    request = {
        "samples": 5,
        "duration": 16,
        "max_iteration": 1000,
        "max_penalty": 500,
        "max_delta": 5,
        "joints": [
            {"start": 0.2, "end": 0.7, "lower_joint_limit": -0.4, "upper_joint_limit": 1.1, "min_velocity": -0.1,
             "max_velocity": 0.1},
            {"start": 0.4, "end": 0.9, "lower_joint_limit": -0.4, "upper_joint_limit": 1.1, "min_velocity": -0.1,
             "max_velocity": 0.1},
        ]
    }
    # db.insert(request)
    # db.drop()
    # result = (db.find({"is_collision_free": True}))
    # result = (db.find({"num_iterations": 90}))
    result = (db.find({"type": "kuka_only_random_trust_region"}))
    # result = (db.find({}))
    # print result
    d = []
    for i in result:
        d.append(i["num_qp_iterations"])
        print i["solver_config"]["trust_region_size"]
    print "len: ", len(d)

    db.remove_item({"solver_config.trust_region_size": 0.2})



if __name__ == '__main__':
    main()