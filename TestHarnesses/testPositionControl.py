import matplotlib.pyplot as plt
import numpy as np

import sys
sys.path.append('..')

from ece163.Quadrotor import QuadrotorModel
from ece163.Quadrotor import QuadrotorPhysicalConstants
from ece163.Quadrotor import QuadrotorController

dt = 0.01
t = 0
x = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u = np.array([4000, 4000, 4000, 4000])

quad = QuadrotorPhysicalConstants.quad
Quadrotor = QuadrotorModel.QuadrotorModel(quad, dt)

traj = QuadrotorController.Trajectory(x, np.array([0, 0, 1]))
env = QuadrotorController.Environment(x, u)
PC = QuadrotorController.PositionController(dT = dt)

t_hist = []
x_hist = []
u_hist = []

for i in range(200):
    t_hist.append(t)
    x_hist.append(x)
    u_hist.append(u)

    x = Quadrotor.update(t, x, u)
    traj = QuadrotorController.Trajectory(x, np.array([0, 0, 1]))
    env = QuadrotorController.Environment(x, u)
    pos, vel, roll, pitch, yaw = PC.control(env, traj, t)
    t += dt

    print("Commands: ", roll, pitch, yaw)

x_hist = np.array(x_hist)
# u_hist = np.array(u_hist)

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(x_hist[:, 0], x_hist[:, 1], abs(x_hist[:, 2]))
# ax.set_title('Position')
# ax.set_xlabel('North [m]')
# ax.set_ylabel('East [m]')
# ax.set_zlabel('Down [m]')
# ax.set_xlim(-1, 1)
# ax.set_ylim(-1, 1)
# plt.grid()

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

# plt.show()