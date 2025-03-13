import matplotlib.pyplot as plt
import numpy as np

import sys
sys.path.append('..')

from ece163.Quadrotor import QuadrotorModel
from ece163.Quadrotor import QuadrotorPhysicalConstants
from ece163.Quadrotor import FlightControllerSystem

dT = 0.01
t = 0
x = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u = np.array([0, 0, 0, 0])

waypoint = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])

quad = QuadrotorPhysicalConstants.quad
Quadrotor = QuadrotorModel.QuadrotorModel(quad, x, dT)

FSC = FlightControllerSystem.FlightControllerSystem(x, waypoint, dT, True)

for i in range(int(30/dT)):

    if i*dT > 1:
        waypoint = np.array([0, 0, -1.5, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0])

    try:
        pose_refout, u = FSC.update(x, waypoint)
        x = Quadrotor.update(x, u)
    except:
        print("Quadrotor.update() failed")
        break

    t += dT

FSC.plotScope(pos_state=False, pos_ref=False, orientation_state=False, orientation_ref=False,
              control=False, thrust=False, controllerTuning = True)

