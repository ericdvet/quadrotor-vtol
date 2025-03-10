"""
Author: Eric Vetha (evetha@ucsc.edu)

Test Harness for Lab 5
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

import ece163.Sensors.SensorsModel as SensorsModel

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

control = VehicleClosedLoopControl.VehicleClosedLoopControl(useSensors = True)
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

accel_x_true, accel_x_noisy = [], []
accel_y_true, accel_y_noisy = [], []
accel_z_true, accel_z_noisy = [], []

gyro_x_true, gyro_x_noisy = [], []
gyro_y_true, gyro_y_noisy = [], []
gyro_z_true, gyro_z_noisy = [], []

baro_true, baro_noisy = [], []
pitot_true, pitot_noisy = [], []

gps_n_true, gps_n_noisy = [], []
gps_e_true, gps_e_noisy = [], []
gps_d_true, gps_d_noisy = [], []
gps_sog_true, gps_sog_noisy = [], []
gps_cog_true, gps_cog_noisy = [], []

mag_x_true, mag_x_noisy = [], []
mag_y_true, mag_y_noisy = [], []
mag_z_true, mag_z_noisy = [], []

for i in range(n_steps):
    if i < n_steps // 2:
        control.Update(referenceCommands_turn)
    else:
        control.Update(referenceCommands_climb)

    state = control.getVehicleState()

    sensor = control.getSensorsModel()
    
    accel_x_true.append(sensor.getSensorsTrue().accel_x)
    accel_x_noisy.append(sensor.getSensorsNoisy().accel_x)
    accel_y_true.append(sensor.getSensorsTrue().accel_y)
    accel_y_noisy.append(sensor.getSensorsNoisy().accel_y)
    accel_z_true.append(sensor.getSensorsTrue().accel_z)
    accel_z_noisy.append(sensor.getSensorsNoisy().accel_z)

    gyro_x_true.append(sensor.getSensorsTrue().gyro_x)
    gyro_x_noisy.append(sensor.getSensorsNoisy().gyro_x)
    gyro_y_true.append(sensor.getSensorsTrue().gyro_y)
    gyro_y_noisy.append(sensor.getSensorsNoisy().gyro_y)
    gyro_z_true.append(sensor.getSensorsTrue().gyro_z)
    gyro_z_noisy.append(sensor.getSensorsNoisy().gyro_z)

    baro_true.append(sensor.getSensorsTrue().baro)
    baro_noisy.append(sensor.getSensorsNoisy().baro)
    pitot_true.append(sensor.getSensorsTrue().pitot)
    pitot_noisy.append(sensor.getSensorsNoisy().pitot)

    gps_n_true.append(sensor.getSensorsTrue().gps_n)
    gps_n_noisy.append(sensor.getSensorsNoisy().gps_n) 
    gps_e_true.append(sensor.getSensorsTrue().gps_e)
    gps_e_noisy.append(sensor.getSensorsNoisy().gps_e)
    gps_d_true.append(sensor.getSensorsTrue().gps_alt)
    gps_d_noisy.append(sensor.getSensorsNoisy().gps_alt)
    gps_sog_true.append(sensor.getSensorsTrue().gps_sog)
    gps_sog_noisy.append(sensor.getSensorsNoisy().gps_sog)
    gps_cog_true.append(sensor.getSensorsTrue().gps_cog)
    gps_cog_noisy.append(sensor.getSensorsNoisy().gps_cog)

    mag_x_true.append(sensor.getSensorsTrue().mag_x)
    mag_x_noisy.append(sensor.getSensorsNoisy().mag_x)
    mag_y_true.append(sensor.getSensorsTrue().mag_y)
    mag_y_noisy.append(sensor.getSensorsNoisy().mag_y)
    mag_z_true.append(sensor.getSensorsTrue().mag_z)
    mag_z_noisy.append(sensor.getSensorsNoisy().mag_z)

    time.append(i * dt)

plt.figure()
plt.title("Accelerometer Data: True vs Noisy")
plt.subplot(3, 1, 1)
plt.plot(time, accel_x_noisy, label="Noisy Accel X")
plt.plot(time, accel_x_true, label="True Accel X")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.subplot(3, 1, 2)
plt.plot(time, accel_y_noisy, label="Noisy Accel Y")
plt.plot(time, accel_y_true, label="True Accel Y")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.subplot(3, 1, 3)
plt.plot(time, accel_z_noisy, label="Noisy Accel Z")
plt.plot(time, accel_z_true, label="True Accel Z")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.legend()

plt.figure()
plt.title("Gyroscope Data: True vs Noisy")
plt.subplot(3, 1, 1)
plt.plot(time, gyro_x_noisy, label="Noisy Gyro X")
plt.plot(time, gyro_x_true, label="True Gyro X")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.subplot(3, 1, 2)
plt.plot(time, gyro_y_noisy, label="Noisy Gyro Y")
plt.plot(time, gyro_y_true, label="True Gyro Y")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.subplot(3, 1, 3)
plt.plot(time, gyro_z_noisy, label="Noisy Gyro Z")
plt.plot(time, gyro_z_true, label="True Gyro Z")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.legend()

plt.figure()
plt.title("Barometer Data: True vs Noisy")
plt.plot(time, baro_noisy, label="Noisy Barometer")
plt.plot(time, baro_true, label="True Barometer")
plt.xlabel("Time (s)")
plt.ylabel("Pressure (Pa)")
plt.legend()

plt.figure()
plt.title("Pitot Tube Data: True vs Noisy")
plt.plot(time, pitot_noisy, label="Noisy Pitot Tube")
plt.plot(time, pitot_true, label="True Pitot Tube")
plt.xlabel("Time (s)")
plt.ylabel("Pressure (Pa)")
plt.legend()

plt.figure()
plt.title("GPS Data: True vs Noisy")
plt.subplot(3, 1, 1)
plt.plot(time, gps_n_noisy, label="Noisy GPS North")
plt.plot(time, gps_n_true, label="True GPS North")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.subplot(3, 1, 2)
plt.plot(time, gps_e_noisy, label="Noisy GPS East")
plt.plot(time, gps_e_true, label="True GPS East")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.subplot(3, 1, 3)
plt.plot(time, gps_d_noisy, label="Noisy GPS Altitude")
plt.plot(time, gps_d_true, label="True GPS Altitude")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.legend()

plt.figure()
plt.title("GPS Speed Over Ground Data: True vs Noisy")
plt.plot(time, gps_sog_noisy, label="Noisy GPS Speed Over Ground")
plt.plot(time, gps_sog_true, label="True GPS Speed Over Ground")
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.legend()

plt.figure()
plt.title("GPS Course Over Ground Data: True vs Noisy")
plt.plot(time, gps_cog_noisy, label="Noisy GPS Course Over Ground")
plt.plot(time, gps_cog_true, label="True GPS Course Over Ground")
plt.xlabel("Time (s)")
plt.ylabel("Course (rad)")
plt.legend()

plt.figure()
plt.title("Magnetometer Data: True vs Noisy")
plt.subplot(3, 1, 1)
plt.plot(time, mag_x_noisy, label="Noisy Mag X")
plt.plot(time, mag_x_true, label="True Mag X")
plt.xlabel("Time (s)")
plt.ylabel("Magnetic Field (T)")
plt.subplot(3, 1, 2)
plt.plot(time, mag_y_noisy, label="Noisy Mag Y")
plt.plot(time, mag_y_true, label="True Mag Y")
plt.xlabel("Time (s)")
plt.ylabel("Magnetic Field (T)")
plt.subplot(3, 1, 3)
plt.plot(time, mag_z_noisy, label="Noisy Mag Z")
plt.plot(time, mag_z_true, label="True Mag Z")
plt.xlabel("Time (s)")
plt.ylabel("Magnetic Field (T)")
plt.legend()
plt.show()



