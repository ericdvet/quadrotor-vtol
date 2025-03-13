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

TESTING_MODE = "TAKE OFF"
TESTING_MODE = "HOVER"

if TESTING_MODE == "TAKE OFF":

    for i in range(200):
        t_hist.append(t)
        x_hist.append(x)
        u_hist.append(u)

        x = Quadrotor.update(t, x, u)
        t += dt

    x_hist = np.array(x_hist)
    u_hist = np.array(u_hist)

elif TESTING_MODE == "HOVER":

    for i in range(250):
        
        if abs(x[2]) < 1:
            u = np.array([4000, 4000, 4000, 4000])
        else:
            u = np.array([1500, 1000, 1000, 1000])

        try:
            x = Quadrotor.update(t, x, u)
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
ax.set_zlim(0, 5)
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

# plt.figure()
# plt.plot(t_hist, x_hist[:, 3:6])
# plt.title('Attitude')
# plt.legend(['Yaw', 'Pitch', 'Roll'])
# plt.xlabel('Time [s]')
# plt.ylabel('Angle [rad]')
# plt.grid()

# plt.figure()
# plt.title("Position over time")
# plt.plot(t_hist, x_hist[:, 0], label="North")
# plt.plot(t_hist, x_hist[:, 1], label="East")
# plt.plot(t_hist, x_hist[:, 2], label="Down")
# plt.legend()
# plt.xlabel("Time [s]")
# plt.ylabel("Position [m]")
# plt.grid()

# plt.figure()
# plt.title("Control Inputs over time")
# plt.plot(t_hist, u_hist[:, 0], label="Front Left")
# plt.plot(t_hist, u_hist[:, 1], label="Front Right")
# plt.plot(t_hist, u_hist[:, 2], label="Back Left")
# plt.plot(t_hist, u_hist[:, 3], label="Back Right")
# plt.legend()
# plt.xlabel("Time [s]")
# plt.ylabel("Control Inputs")
# plt.grid()

plt.show()