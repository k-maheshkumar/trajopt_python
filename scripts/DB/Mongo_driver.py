from pymongo import Connection


class MongoDriver():
    def __init__(self):
        # self.client = MongoClient('localhost', 27017)
        self.client = Connection()
        self.db_collection = self.client["trajectory_planner"]
        self.db = self.db_collection.request

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
        # self.insert(request)
        self.find()

    def find(self):
        peeps = self.db.find()
        for p in peeps:
            print p


    def insert(self, request):
        self.db.insert(request)


def main():

    db = MongoDriver()

if __name__ == '__main__':
    main()