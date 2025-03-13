import matplotlib.pyplot as plt
import numpy as np

import sys
sys.path.append('..')

from ece163.Quadrotor import QuadrotorModel
from ece163.Quadrotor import QuadrotorPhysicalConstants
from matplotlib.animation import FuncAnimation

dt = 0.01
t = 0
x = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u = np.array([4000, 4000, 4000, 4000])

quad = QuadrotorPhysicalConstants.quad
Quadrotor = QuadrotorModel.QuadrotorModel(quad, dt)

t_hist = []
x_hist = []
u_hist = []


for i in range(200):

    try:
        x = Quadrotor.update(x, u)
    except:
        print("Quadrotor.update() failed")

    t += dt

    t_hist.append(t)
    x_hist.append(x)
    u_hist.append(u)

x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title('Position')
ax.set_xlabel('North [m]')
ax.set_ylabel('East [m]')
ax.set_zlabel('Down [m]')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(0, 15)
line, = ax.plot([], [], [], lw=2)

def init():
    line.set_data([], [])
    line.set_3d_properties([])
    return line,

def update(frame):
    line.set_data(x_hist[:frame, 0], x_hist[:frame, 1])
    line.set_3d_properties(abs(x_hist[:frame, 2]))
    return line,

ani = FuncAnimation(fig, update, frames=len(t_hist), init_func=init, blit=True)

plt.grid()

plt.show()