from pymongo import MongoClient


class MongoDriver():
    def __init__(self, db_name="Trajectory_planner_results", host='localhost', port=27017):
        self.client = MongoClient(host, port)
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