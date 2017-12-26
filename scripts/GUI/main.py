from scripts.GUI import TrajPlanner1
from PyQt4 import QtGui
import sys

if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    window = TrajPlanner1.PlannerGui(25)
    window.show()
    sys.exit(app.exec_())