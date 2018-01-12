from scripts.GUI import TrajPlanner
from PyQt4 import QtGui
import sys

if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    window = TrajPlanner.PlannerGui(verbose="DEBUG", file_log=False)
    window.show()
    sys.exit(app.exec_())