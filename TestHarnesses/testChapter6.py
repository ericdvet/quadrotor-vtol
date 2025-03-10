"""
Author: Eric Vetha (evetha@ucsc.edu)

Modified Test Harness for Homework 4, Question 5
"""

import sys
import math
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("..")  # Python is amazing, no?

import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule
import ece163.Controls.VehicleClosedLoopControl as VehicleClosedLoopControl
import ece163.Containers.States as States

testHarnessMode = True

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

control = VehicleClosedLoopControl.VehicleClosedLoopControl()
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

time_data = []
aileron_data, rudder_data, roll_rate_data, roll_angle_data, course_data = [], [], [], [], []
elevator_data, throttle_data, pitch_rate_data, pitch_angle_data, altitude_data = [], [], [], [], []

try:
    csv_data = np.loadtxt("testChapter6.csv", delimiter=",", skiprows=1)
    time_ground_truth = csv_data[:, 0]
    elevator_ground_truth = csv_data[:, 1]
    throttle_ground_truth = csv_data[:, 2]
    pitch_rate_ground_truth = csv_data[:, 3]
    pitch_angle_ground_truth = csv_data[:, 4]
    altitude_ground_truth = csv_data[:, 5]
    passed_tests = 0
except:
    print("testChapter6.csv not found. Please pull the latest version of the repository from Gitlab.")
    quit()

for i in range(n_steps):
    if i < n_steps // 2:
        control.Update(referenceCommands_turn)
    else:
        control.Update(referenceCommands_climb)

    state = control.getVehicleState()

    time_data.append(i * dt)
    temp = control.getVehicleControlSurfaces()
    aileron_data.append(math.degrees(temp.Aileron))
    rudder_data.append(math.degrees(temp.Rudder))
    roll_rate_data.append(math.degrees(state.p))
    roll_angle_data.append(math.degrees(state.roll))
    course_data.append(math.degrees(state.chi))

    elevator_data.append(math.degrees(temp.Elevator))
    throttle_data.append(temp.Throttle)
    pitch_rate_data.append(math.degrees(state.q))
    pitch_angle_data.append(math.degrees(state.pitch))
    altitude_data.append(-state.pd) # -state.pd because the altitude is negative

    if testHarnessMode == True:
        if (elevator_data[i] != elevator_ground_truth[i] or
            throttle_data[i] != throttle_ground_truth[i] or
            pitch_rate_data[i] != pitch_rate_ground_truth[i] or
            pitch_angle_data[i] != pitch_angle_ground_truth[i] or
            altitude_data[i] != altitude_ground_truth[i]):
            print("Test Run " + str(i) + " failed")
        else:
            print("Test Run " + str(i) + " passed")
            passed_tests += 1

plt.figure()
plt.plot(time_data, aileron_data, label="δa")
plt.plot(time_data, rudder_data, label="δr")
plt.plot(time_data, roll_rate_data, label="p")
plt.plot(time_data, roll_angle_data, label="ϕ")
plt.plot(time_data, course_data, label="χ")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.legend()

plt.figure()
plt.plot(time_data, elevator_data, label="δe")
plt.plot(time_data, throttle_data, label="δT")
plt.plot(time_data, pitch_rate_data, label="q")
plt.plot(time_data, pitch_angle_data, label="θ")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg or deg/s)")
plt.legend()

plt.figure()
plt.plot(time_data, altitude_data, label="Altitude (h)")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.legend()

if testHarnessMode == False:
    plt.show()
else:
    print("Passed tests: " + str(passed_tests) + "/" + str(n_steps))
    if passed_tests == n_steps:
        print("All tests passed!")
    else:
        print("Some tests failed.")