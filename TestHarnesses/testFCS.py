import matplotlib.pyplot as plt
import numpy as np

import sys
sys.path.append('..')

from ece163.Quadrotor import QuadrotorModel
from ece163.Quadrotor import QuadrotorPhysicalConstants
from ece163.Quadrotor import FlightControllerSystem

dt = 0.01
t = 0
x = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u = np.array([0, 0, 0, 0])

waypoint = np.array([0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0])

quad = QuadrotorPhysicalConstants.quad
Quadrotor = QuadrotorModel.QuadrotorModel(quad, dt)

FSC = FlightControllerSystem.FlightControllerSystem(x, waypoint)

t_hist = []

px_hist = []
py_hist = []
pz_hist = []

yaw_hist = []
pitch_hist = []
roll_hist = []

u1 = []
u2 = []
u3 = []
u4 = []

for i in range(200):

    try:
        x = Quadrotor.update(t, x, u)
        pose_refout, u = FSC.update(x, waypoint)
    except:
        print("Quadrotor.update() failed")
        break

    t += dt

    t_hist.append(t)
    px_hist.append(x[0])
    py_hist.append(x[1])
    pz_hist.append(abs(x[2]))
    yaw_hist.append(x[6])
    pitch_hist.append(x[7])
    roll_hist.append(x[8])
    u1.append(u[0])
    u2.append(u[1])
    u3.append(u[2])
    u4.append(u[3])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(px_hist, py_hist, pz_hist, label='Quadrotor Path')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.legend()
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)

fig, axs = plt.subplots(2, 1, figsize=(10, 10))

# Plot all control inputs u in one plot
axs[0].plot(t_hist, u1, label='u1')
axs[0].plot(t_hist, u2, label='u2')
axs[0].plot(t_hist, u3, label='u3')
axs[0].plot(t_hist, u4, label='u4')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Control Inputs')
axs[0].legend()
axs[0].set_title('Control Inputs vs Time')

# Plot all angles (yaw, pitch, roll) in one plot
axs[1].plot(t_hist, yaw_hist, label='Yaw')
axs[1].plot(t_hist, pitch_hist, label='Pitch')
axs[1].plot(t_hist, roll_hist, label='Roll')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Angles (rad)')
axs[1].legend()
axs[1].set_title('Angles vs Time')

plt.tight_layout()

plt.show()