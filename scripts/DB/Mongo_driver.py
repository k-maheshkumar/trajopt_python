from pymongo import Connection
from pymongo import MongoClient


class MongoDriver():
    def __init__(self, db_name, host='localhost', port=27017):
        self.client = MongoClient(host, port)
        # self.client = Connection()
        self.db_collection = self.client[db_name]
        self.db = self.db_collection.db_name

    def find(self, query):
        return self.db.find_one(query)

    def insert(self, request):

        self.db.insert(request)


def main():

    db = MongoDriver("trajectory_planner")
    request = {
        "samples": 5,
        "duration": 6,
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
    print(db.find({"samples": 5}))


if __name__ == '__main__':
    main()