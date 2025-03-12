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

waypoint = np.array([0, 0, -1000])

quad = QuadrotorPhysicalConstants.quad
Quadrotor = QuadrotorModel.QuadrotorModel(quad, dt)

MV = QuadrotorController.ManeuverController(dT = dt)

thrust_hist = []
distance_hist = []
t_hist = []

for i in range(200):

    x = Quadrotor.update(t, x, u)

    m = QuadrotorController.Environment(x, u)
    traj = QuadrotorController.Trajectory(x, waypoint)

    motorPitch, motorRoll, motorYaw, motorThrust, CMD_w = MV.control(m, traj)

    thrust_hist.append(motorThrust)
    distance_hist.append(abs(waypoint[2] - x[2]))
    t_hist.append(t)

    t += dt

plt.figure()
plt.plot(t_hist, thrust_hist / np.linalg.norm(thrust_hist))
plt.plot(t_hist, distance_hist / np.linalg.norm(distance_hist))
plt.title("Thrust vs Distance Away from Object")
plt.xlabel("Distance Away from Object [m]")
plt.ylabel("Thrust [N]")
plt.legend(["Thrust", "Distance"])
plt.grid()
plt.show()
