from pymongo import Connection
from pymongo import MongoClient


class MongoDriver():
    def __init__(self, db_name, host='localhost', port=27017):
        self.client = MongoClient(host, port)
        # self.client = Connection()
        self.db_collection = self.client[db_name]
        self.db = self.db_collection.db_name

    def find_one(self, query):
        return self.db.find_one(query)

    def find(self, query):
        return self.db.find(query)

    def insert(self, request):
        print request
        self.db.insert(request)
        print "saved request into database . . . . . . .  ."

    def say_hello(self):
        print "hello . . ."

    def drop(self):
        self.db.drop()


def main():

    db = MongoDriver("trajectory_planner")
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
    result = (db.find({}))
    # print result
    for i in result:
        print i["samples"]
        # print i["trajectory"]


if __name__ == '__main__':
    main()