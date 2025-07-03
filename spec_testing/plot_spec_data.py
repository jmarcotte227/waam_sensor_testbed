import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.animation as animation

data_dir = '../../wst_data/_weld_100IPM20250703-140141/'
wavelengths = np.loadtxt(f'{data_dir}spec_wavelengths.csv', delimiter=',', skiprows=1)[0,1:]
counts = np.loadtxt(f'{data_dir}spec_counts.csv', delimiter=',', skiprows=1)

print(wavelengths.shape)
print(counts.shape)
Y_LIM = [10,3640]
i = 0
length = counts.shape[0]
print(wavelengths[Y_LIM[0]:Y_LIM[1]].shape)
print(counts[0,Y_LIM[0]:Y_LIM[1]].shape)

fig,ax = plt.subplots(1,1)
ax.set_ylim([-5,500])
line, = ax.plot(wavelengths[Y_LIM[0]:Y_LIM[1]], counts[i,Y_LIM[0]:Y_LIM[1]])

def animate(i):
    line.set_ydata(counts[i,Y_LIM[0]:Y_LIM[1]])
    return line,

ani = animation.FuncAnimation(
    fig, animate, interval=10, blit=True, frames=length)

plt.show()
