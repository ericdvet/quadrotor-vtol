"""
Description: This module contains the QuadrotorController class, which is used to control the quadrotor's position and orientation. We use a PID controller based on the work in "Modelling and PID Controller Design for a Quadrotor Unmanned Air Vehicle" (Salih et al. 20??) 
"""

# delete these notes when done

from . import QuadrotorPhysicalConstants
import numpy as np
from simple_pid import PID
import control as ctrl

class Trajectory:

    def __init__(self, x, x_target):
        
        # We need to constantly output the trajectory's yaw, position, and velocity
        # which means that we need to constantly update the trajectory because
        # the derivative is required for the controller

        self.yaw = np.arctan2(x_target[1] - x[1], x_target[0] - x[0])
        self.position = x_target
        self.velocity = np.zeros(3)
        self.t = 0
    
    def update(self,  x, x_target, dt):

        self.yaw = np.arctan2(x_target[1] - x[1], x_target[0] - x[0])
        self.t += dt
        self.velocity = (x[:3] - self.position) / dt
        self.position = x_target

# Done with reading that state vector
class Environment():

    def __init__(self, x, u):
        
        self.px = x[0]
        self.py = x[1]
        self.pz = x[2]

        self.vx = x[6]
        self.vy = x[7]
        self.vz = x[8]

        self.roll = x[3]
        self.pitch = x[4]
        self.yaw = x[5]

        self.Prop1 = u[0]
        self.Prop2 = u[1]
        self.Prop3 = u[2]
        self.Prop4 = u[3]

        self.Prop1_w = 0    # Where am I supposed to get this from?
        self.Prop2_w = 0
        self.Prop3_w = 0
        self.Prop4_w = 0

# Position control should take in the roll pitch yaw of the quadrotor in the body frame,
# trajectory position, and the quadrotor's position in the world frame
# It'll do some shit with that and output the pitch and roll commands
# The yaw commands should just be in trajectory obviously.
# Reference  position and reference velocity assuming we want to propogate it through her
# would come straight through trajectory position and velocity
# TODO: Does this need to be a class? FFS it does

class PositionController:
    def __init__(self, dT):

        self.dT = dT

        self.kp_xy2ypr = 8          # This shouldn't be hardcoded
        self.ki_xy2ypr = 0.04
        self.kd_xy2ypr = 3.2
        self.filtD_xy2ypr = 100
        self.filter_cnst = 0.005
        self.err2rp = 2.4
        
        self.XY2YPR = PID(Kp = self.kp_xy2ypr, 
                          Ki = self.ki_xy2ypr, 
                          Kd = self.kd_xy2ypr)
        self.filter = LowPassFilter(self.filter_cnst, dT)

    def control(self, m: Environment, traj: Trajectory):
        # Body Frame
        R_BI = calculateRBI(m.roll, m.pitch, m.yaw)

        # Compute position error
        pos_error = traj.position - np.array([m.px, m.py, m.pz])

        # PID
        px = self.XY2YPR(pos_error[0])
        py = self.XY2YPR(pos_error[1])
        pz = self.XY2YPR(pos_error[2])

        i = R_BI @ np.array([px, py, pz])

        o = np.zeros(3)

        # Filter
        o[0] = self.filter.filter(i[0])
        o[1] = self.filter.filter(i[1])
        o[2] = self.filter.filter(i[2]) # not used
        
        o[0] = 1 / 9.81 * self.err2rp * i[0]
        o[1] = 1 / 9.81 * self.err2rp * i[1]

        o[0] = np.clip(o[0], -np.pi/3, np.pi/3)
        o[1] = np.clip(o[1], -np.pi/3, np.pi/3)

        pitch = o[0]
        roll = o[1]

        pos = traj.position
        vel = traj.velocity
        yaw = traj.yaw

        return pos, vel, roll, pitch, yaw

class LowPassFilter:
    def __init__(self, filt_const, dt):
        self.filt_const = filt_const
        self.dt = dt
        self.alpha = dt / (filt_const + dt)
        self.prev_output = None 
    
    def filter(self, input):
        if self.prev_output is None:
            self.prev_output = input 
        
        output = self.alpha * input + (1 - self.alpha) * self.prev_output
        
        self.prev_output = output
        
        return output

def calculateRBI(phi, theta, psi):
    """
    Calculates R_BI
    """

    R_BI = np.array([
        [np.cos(theta) * np.cos(psi),                                               np.cos(theta) * np.sin(psi),                                            -np.sin(theta)],
        [-np.cos(phi) * np.sin(psi) + np.sin(phi) * np.sin(theta) * np.cos(psi),    np.cos(phi) * np.cos(psi) + np.sin(phi) * np.sin(theta) * np.sin(psi),  np.sin(phi) * np.cos(theta)],
        [np.sin(phi) * np.sin(psi) + np.cos(phi) * np.sin(theta) * np.cos(psi),     -np.sin(phi) * np.cos(psi) + np.cos(phi) * np.sin(theta) * np.sin(psi), np.cos(phi) * np.cos(theta)]
    ])
    return R_BI

class AltitideAndYPRControl():
    
    def __init__(self, dT):
        
        self.dT = dT    

        self.pitchCMD, self.rollCMD, self.yawCMD = 0, 0, 0

        altitude_filtM = 0.05
        yaw_filtM = 0.01
        pz_filtM = 0.01

        self.FilterPitch = LowPassFilter(altitude_filtM, dT)
        self.FilterRoll = LowPassFilter(altitude_filtM, dT)
        self.FilterYaw = LowPassFilter(yaw_filtM, dT)
        self.FilterThrust = LowPassFilter(pz_filtM, dT)

        attitude_kp = 128.5
        attitude_ki = 5.9203
        attitude_kd = 156.4
        attitude_f = 1000
        self.PitchPID = PID(Kp = attitude_kp, Ki = attitude_ki, Kd = attitude_kd)
        self.RollPID = PID(Kp = attitude_kp, Ki = attitude_ki, Kd = attitude_kd)

        yaw_kp = 205.61
        yaw_ki = 0.059203
        yaw_kd = 0.782
        yaw_f = 100
        self.YawPID = PID(Kp = yaw_kp, Ki = yaw_ki, Kd = yaw_kd)

        altitude_kp = 0.27
        altitude_ki = 0.07
        altitude_kd = 0.35
        altitude_f = 10000
        self.AltitudePID = PID(Kp = altitude_kp, Ki = altitude_ki, Kd = altitude_kd)
        
    def control(self, m: Environment, ref: Trajectory, pitchCMD = 0, rollCMD = 0, yawCMD = 0):

        self.m = m
        self.ref = ref
        self.pitchCMD, self.rollCMD, self.yawCMD = pitchCMD, rollCMD, yawCMD
        
        # Motor Pitch
        filteredPitch = self.FilterPitch.filter(self.m.pitch)
        pitchPreControl = self.pitchCMD - filteredPitch
        motorPitch = self.PitchPID(pitchPreControl)

        # Motor Roll
        filteredRoll = self.FilterRoll.filter(self.m.roll)
        rollPreControl = self.rollCMD - filteredRoll
        motorRoll = self.RollPID(rollPreControl)

        # Motor Yaw
        filteredYaw = self.FilterYaw.filter(self.m.yaw)
        yawPreControl = self.yawCMD - filteredYaw
        motorYaw = self.YawPID(yawPreControl)

        # Motor Altitude
        pz = self.m.pz
        ref_pz = self.ref.position[2]
        thrustPreControl = ref_pz - pz
        motorThrust = self.AltitudePID(thrustPreControl)

        k_temp = 1363.9 # 2*pi/(propeller.diameter^4 * sqrt(air_rho)) 
        motorThrust = k_temp * motorThrust + 700

        return motorPitch, motorRoll, motorYaw, motorThrust
    
class ManeuverController():

    def __init__(self, dT):
        self.dT = dT

        self.PC = PositionController(dT)
        self.AYPR = AltitideAndYPRControl(dT)
    
    def control(self, m: Environment, Traj: Trajectory):

        self.m = m
        self.Traj = Traj
        
        pos, vel, roll, pitch, yaw = self.PC.control(self.m, self.Traj)
        motorPitch, motorRoll, motorYaw, motorThrust = self.AYPR.control(self.m, self.Traj, pitch, roll, yaw)

        CMD_w1, CMD_w2, CMD_w3, CMD_w4 = self.motorMixer(motorPitch, motorRoll, motorYaw, motorThrust)

        return motorPitch, motorRoll, motorYaw, motorThrust, np.array([
            CMD_w1, CMD_w2, CMD_w3, CMD_w4])

    def motorMixer(self, motorPitch, motorRoll, motorYaw, motorThrust):

        CMD_w1 = motorRoll - motorPitch - motorYaw + motorThrust
        CMD_w2 = -motorRoll - motorPitch + motorYaw + motorThrust
        CMD_w3 = motorRoll + motorPitch + motorYaw + motorThrust
        CMD_w4 = -motorRoll + motorPitch - motorYaw + motorThrust

        return CMD_w1, CMD_w2, CMD_w3, CMD_w4
    
def w2rpm(w):
    prop_1 =  max(min(
        w[0]
        , 6000), -6000)
    prop_2 = max(min(
        w[1]
        , 6000), -6000)
    prop_3 = max(min(
        w[2]
        , 6000), -6000)
    prop_4 = max(min(
        w[3]
        , 6000), -6000)
    return prop_1, prop_2, prop_3, prop_4