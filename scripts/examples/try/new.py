import numpy as np

class A:
    def __init__(self):
        self.d = D()
    def call_back(self, param):

        print param
        return "from A"

    def function(self):
        return self.d.function(self.call_back)

class B:
    def __init__(self):
        pass

    def function(self, callback):
        return callback("from B")


class C:
    def __init__(self):
        pass

    def function(self, callback):
        b = B()

        return b.function(callback)


class D:
    def __init__(self):
        pass
    def function(self, callback):
        c = C()
        return c.function(callback)
if __name__ == '__main__':
    a = A()
    print a.function()
