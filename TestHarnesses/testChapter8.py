import sys
import math
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("..") # python is amazing, no?

import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule
import ece163.Controls.VehicleClosedLoopControl as VehicleClosedLoopControl
import ece163.Containers.States as States
import ece163.Sensors.SensorsModel as SensorsModel
import ece163.Controls.VehicleEstimator as VE
import ece163.Containers.Sensors as Sensors

vehicleEstimator = VE.VehicleEstimator()

print("Vehicle Estimator Test Harness Loading...")

state = States.vehicleState()
state.Va = VPC.InitialSpeed
state.pn = VPC.InitialNorthPosition
state.pe = VPC.InitialEastPosition
state.pd = VPC.InitialDownPosition
state.yaw = VPC.InitialYawAngle

referenceCommands_turn = Controls.referenceCommands()
referenceCommands_turn.commandedCourse = math.radians(30)  # 30 degree control turn

referenceCommands_climb = Controls.referenceCommands()
referenceCommands_climb.commandedAltitude = state.pd - 50

control = VehicleClosedLoopControl.VehicleClosedLoopControl(useSensors=True, useEstimator=True)
controlGains = Controls.controlGains(
    kp_roll=0.8, kd_roll=0.3, ki_roll=0.1,
    kp_sideslip=0.5, ki_sideslip=0.1,
    kp_course=1.0, ki_course=0.3,
    kp_pitch=0.8, kd_pitch=0.2,
    kp_altitude=0.7, ki_altitude=0.2,
    kp_SpeedfromThrottle=0.9, ki_SpeedfromThrottle=0.2,
    kp_SpeedfromElevator=0.6, ki_SpeedfromElevator=0.2
)
control.setControlGains(controlGains)
control.setVehicleState(state)

trimInputs = Inputs.controlInputs()
control.setTrimInputs(trimInputs)

dt = 0.01
T_total = 10  
n_steps = int(T_total / dt)  

time = []

pn_true, pn_est = [], []
pe_true, pe_est = [], []
pd_true, pd_est = [], []
yaw_true, yaw_est = [], []

for i in range(n_steps):
    if i < n_steps // 2:
        control.Update(referenceCommands_turn)
    else:
        control.Update(referenceCommands_climb)

    state = control.getVehicleState()
    vehicleEstimator.setEstimatedState(state)
    vehicleEstimator.update()

    estimatedState = vehicleEstimator.getEstimatedState()

    pn_true.append(state.pn)
    pn_est.append(estimatedState.pn)
    pe_true.append(state.pe)
    pe_est.append(estimatedState.pe)
    pd_true.append(state.pd)
    pd_est.append(estimatedState.pd)
    yaw_true.append(state.yaw)
    yaw_est.append(estimatedState.yaw)

    time.append(i * dt)

plt.figure()
plt.title("Position North: True vs Estimated")
plt.plot(time, pn_true, label="True Position North")
plt.plot(time, pn_est, label="Estimated Position North")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()

plt.figure()
plt.title("Position East: True vs Estimated")
plt.plot(time, pe_true, label="True Position East")
plt.plot(time, pe_est, label="Estimated Position East")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()

plt.figure()
plt.title("Position Down: True vs Estimated")
plt.plot(time, pd_true, label="True Position Down")
plt.plot(time, pd_est, label="Estimated Position Down")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()

plt.figure()
plt.title("Yaw: True vs Estimated")
plt.plot(time, yaw_true, label="True Yaw")
plt.plot(time, yaw_est, label="Estimated Yaw")
plt.xlabel("Time (s)")
plt.ylabel("Yaw (rad)")
plt.legend()

print("Vehicle Estimator Test Harness Complete, please see plots for results.")

plt.show()
