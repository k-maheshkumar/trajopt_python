import scipy.interpolate as sp
import numpy as np
import matplotlib.pyplot as plt

x_given = np.linspace(0, 10, 5)
y_given = np.linspace(10, 20, 5)

x_p = np.linspace(0, 10, 10)


p3 = np.polyfit(x_given, y_given, 3)
y_p = np.polyval(p3, x_p)


plt.plot(x_given, y_given, 'o')
plt.plot(x_p, y_p, '-')

plt._show()
