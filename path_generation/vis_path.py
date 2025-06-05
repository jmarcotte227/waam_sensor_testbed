import numpy as np
import matplotlib.pyplot as plt

PATH = 'wall.csv'
path = np.loadtxt(f"paths/{PATH}", delimiter=',')
print(path)

fig,ax = plt.subplots(1,1)
ax.plot(path[:,0], path[:,2])
plt.show()
