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

t_hist = []

px_hist = []
py_hist = []
pz_hist = []

prop1_hist = []
prop2_hist = []
prop3_hist = []
prop4_hist = []

for i in range(200):

    t_hist.append(t)
    px_hist.append(x[0])
    py_hist.append(x[1])
    pz_hist.append(abs(x[2]))

    try:
        x = Quadrotor.update(t, x, u)
    except:
        print("Quadrotor.update() failed")

    m = QuadrotorController.Environment(x, u)
    traj = QuadrotorController.Trajectory(x, waypoint)

    motorPitch, motorRoll, motorYaw, motorThrust, CMD_w = MV.control(m, traj)
    # QuadrotorController.w2rpm(CMD_w)
    prop1, prop2, prop3, prop4 = QuadrotorController.w2rpm(CMD_w)
    u = np.array([prop1, prop2, prop3, prop4])

    t += dt

    print(u[0])
    prop1_hist.append(u[0])
    prop2_hist.append(u[1])
    prop3_hist.append(u[2])
    prop4_hist.append(u[3])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(px_hist, py_hist, pz_hist, label='Quadrotor Path')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('3D Trajectory of Quadrotor')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.legend()

fig, ax = plt.subplots()
ax.plot(t_hist, prop1_hist, label='Propeller 1')
ax.plot(t_hist, prop2_hist, label='Propeller 2')
ax.plot(t_hist, prop3_hist, label='Propeller 3')
ax.plot(t_hist, prop4_hist, label='Propeller 4')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Propeller Force (N)')
ax.set_title('Propeller Forces Over Time')
ax.legend()


plt.show()

