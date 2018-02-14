from scripts.GUI import TrajPlanner
from PyQt4 import QtGui
import sys
from scripts.simulation.SimulationWorld import SimulationWorld
if __name__ == '__main__':
    simulation = SimulationWorld()
    app = QtGui.QApplication(sys.argv)
    window = TrajPlanner.PlannerGui(verbose="DEBUG", file_log=False, simulation=simulation)
    window.show()
    sys.exit(app.exec_())

