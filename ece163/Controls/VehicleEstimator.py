import math
from ..Containers import Controls
from ..Containers import Sensors
from ..Containers import States
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleDynamicsModel as VDM
from ..Sensors import SensorsModel
from ..Utilities import MatrixMath as MM
from ..Utilities import Rotations as R
import numpy as np
from scipy.linalg import expm

class LowPassFilter:
    """
    Class that implements a low-pass filter with a given cutoff frequency and time step.
    
    :param dT: The time step [s], defaults to VehiclePhysicalConstants.dT
    :param cutoff: The filter's cutoff frequency in [Hz], defaults to 1 Hz
    :return: none
    """
    
    def __init__(self, dT=VPC.dT, cutoff=1):
        """
        Function to initialize the LowPassFilter.
        
        :param dT: The time step [s], defaults to VehiclePhysicalConstants.dT
        :param cutoff: The filter's cutoff frequency in [Hz], defaults to 1 Hz
        :return: none
        """

        self.dT = dT
        self.alpha = 2 * math.pi * cutoff
        self.output = 0
    
    def reset(self):
        """
        Function to reset the internal state of the low-pass filter.
        
        :return: none
        """

        self.output = 0
        
    
    def update(self, input):
        """
        Function to update the low-pass filter based on some input and the previous output.
        
        :param input: The new input
        :return: output, the updated output
        """

        self.output = math.exp(-self.alpha * self.dT) * self.output + (1 - math.exp(-self.alpha * self.dT)) * input
        return self.output
        
    
class VehicleEstimator:
    """
    Class that implements a vehicle state estimator using various filters and sensor data.
    
    :param dT: Update time step, defaults to VehiclePhysicalConstants.dT
    :param gains: The gains for the estimation filters, defaults to Controls.VehicleEstimatorGains()
    :param sensorsModel: The SensorsModel class object, defaults to SensorsModel.SensorsModel()
    :return: none
    """
    
    def __init__(self, dT=VPC.dT, gains=Controls.VehicleEstimatorGains(), sensorsModel=SensorsModel.SensorsModel()):
        """
        Function to initialize the VehicleEstimator.
        
        :param dT: Update time step, defaults to VehiclePhysicalConstants.dT
        :param gains: The gains for the estimation filters, defaults to Controls.VehicleEstimatorGains()
        :param sensorsModel: The SensorsModel class object, defaults to SensorsModel.SensorsModel()
        :return: none
        """

        self.dT = dT
        self.gains = gains
        self.sensorsModel = sensorsModel

        self.estimatedState = States.vehicleState(pd=VPC.InitialDownPosition, dcm=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.baroFilter = LowPassFilter(dT=self.dT, cutoff=self.gains.lowPassCutoff_h)

        self.gyro_bias = [[0], [0], [0]]
        self.pitot_bias = 0  
        self.b_gps_bias = 0
        self.b_dot = 0
        self.chi_bias = 0

        self.Update = self.update
    
    def update(self):
        """
        Wrapper function to update the sensors and then update the estimated state using either complementary filters or Kalman filters.
        
        :return: none
        """

        sensorData = self.sensorsModel.getSensorsTrue()
        
        est_b, est_w, est_R = self.estimateAttitude(sensorData, self.estimatedState)
        self.estimatedState.R = est_R
        self.estimatedState.p = est_w[0][0]
        self.estimatedState.q = est_w[1][0]
        self.estimatedState.r = est_w[2][0]
        self.gyro_bias = est_b
        
        h_est, h_dot, b_gps_prev = self.estimateAltitude(sensorData, self.estimatedState)
        self.estimatedState.pd = -h_est
        self.b_dot = h_dot
        self.b_gps_bias = b_gps_prev
        
        b_Va, Va = self.estimateAirspeed(sensorData, self.estimatedState)
        self.estimatedState.Va = Va
        self.pitot_bias = b_Va
        
        b_x_prev, chi_est = self.estimateCourse(sensorData, self.estimatedState)
        self.estimatedState.chi = chi_est
        self.chi_bias = b_x_prev
        
        self.estimatedState.pn += self.estimatedState.u * self.dT
        self.estimatedState.pe += self.estimatedState.v * self.dT
        self.estimatedState.pd += self.estimatedState.w * self.dT
        self.estimatedState.u += self.estimatedState.p * self.dT
        self.estimatedState.v += self.estimatedState.q * self.dT
        self.estimatedState.w += self.estimatedState.r * self.dT
    
    def estimateAirspeed(self, sensorData = Sensors.vehicleSensors(), estimatedState = States.vehicleState()):
        """
        Function to estimate the vehicle’s airspeed (Va) by combining the estimated attitude (gyroscope and accelerometer) and GPS using a complementary filter.

        Uses the following gains from the class gain attribute: Kp_Va, Ki_Va. Larger values for these gains means you trust the pitot more.
        
        :param sensorData: The noisy sensors data to be filtered
        :param estimatedState: The previous estimate of the state
        :return: b_est, Va, the estimated pitot bias and estimated airspeed
        """

        Va = estimatedState.Va
        b_Va = self.pitot_bias

        Va_pitot = math.sqrt(2* sensorData.pitot / VPC.rho)

        a_x = np.array([[sensorData.accel_x], [sensorData.accel_y], [sensorData.accel_z]]) + np.matrix(estimatedState.R) @ np.array([[0], [0], [9.81]])
        a_x = a_x[0,0]
        
        b_dot_Va = -self.gains.Ki_Va * (Va_pitot - Va)
        b_Va = b_Va + b_dot_Va * self.dT
        Va_dot = a_x - b_Va + self.gains.Kp_Va * (Va_pitot - Va)
        Va = Va + Va_dot * self.dT
        
        return b_Va, Va


    
    def estimateAltitude(self, sensorData=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0, accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0, gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0), estimatedState=States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0, dcm=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])):
        """
        estimateAltitude_Complementary Function to estiamte the vehicle’s altitude (h) by passing barometer data
        through a high-pass filter and GPS data through a low-pass filter and combining them using a complemen-
        tary filter.

        Uses the following gains from the class gain attribute: Kp_h, Ki_h. Larger values for these gains means
        you trust the baro more

        :param sensorData: The noisy sensors data to be filtered
        :param estimatedState: The previous estimate of the state
        :return: h_est, h_dot_est, b_est the estimated altitude, estimated ascent rate, and bias
        """

        h_est = sensorData.gps_alt
        b_h_dot = 0
        b_gps_prev = 0

        b_gps_prev = self.b_gps_bias
        h_baro = (sensorData.baro - VSC.Pground) / VPC.rho / VPC.g0
        h_LPF = self.baroFilter.update(h_baro)

        b_dot_h_dot = -self.gains.Ki_h * (h_LPF + h_est)
        b_h_dot = self.b_dot + b_dot_h_dot * self.dT

        a_i_up = np.matrix(estimatedState.R).T @ np.array([[sensorData.accel_x], [sensorData.accel_y], [sensorData.accel_z]]) + np.array([[0], [0], [9.81]])
        a_i_up = a_i_up[2,0]

        h_dot = a_i_up * self.dT + b_h_dot
        h_est = h_est + (self.gains.Kp_h * (h_LPF - h_est) + h_dot) * self.dT - b_gps_prev

        if self.sensorsModel.tickCounter % (1 / self.sensorsModel.gpsUpdateHz) == self.sensorsModel.dT:
            temp = 2
        
        return h_est, h_dot, b_gps_prev

    def estimateAttitude(self, sensorData=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0, accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0, gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0), estimatedState=States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0, dcm=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])):
        """
        Function to estimate the vehicle’s attitude (R) and angular rates (p, q, r) by combining the gyroscope readings with accelerometer and magnetometer readings using a complementary filter to remove the gyro’s biases and integrate the DCM.

        Uses the following gains from the class gain attribute: Kp_acc, Ki_acc, Kp_mag, Ki_mag. Larger values for these gains means you trust the gyro less and the accelerometer/magnetometer more.
        
        :param sensorData: The noisy sensors data to be filtered
        :param estimatedState: The previous estimate of the state
        :return: est_b, est_w, est_R, the estimated gyro biases, angular rates, and DCM
        """
        
        est_R = np.array(estimatedState.R)  # initialize DCM estimate;
        est_b = np.array(self.gyro_bias)    # initial bias
        
        # Normalize known inertial acceleration
        a_i = np.array([[0], [0], [9.81]])
        a_i = a_i / np.linalg.norm(a_i) if np.linalg.norm(a_i) != 0 else a_i
        a_b = np.array([[sensorData.accel_x], [sensorData.accel_y], [sensorData.accel_z]])
        a_b = a_b / np.linalg.norm(a_b) if np.linalg.norm(a_b) != 0 else a_b

        # Normalize known inertial magnetoration
        h_i = np.array(VSC.magfield)
        h_i = h_i / np.linalg.norm(h_i) if np.linalg.norm(h_i) != 0 else h_i
        h_b = np.array([[sensorData.mag_x], [sensorData.mag_y], [sensorData.mag_z]])
        h_b = h_b / np.linalg.norm(h_b) if np.linalg.norm(h_b) != 0 else h_b

        omega = np.array([[sensorData.gyro_x], [sensorData.gyro_y], [sensorData.gyro_z]])

        # accel
        if not (0.9 * 9.81 < np.linalg.norm(a_b) < 1.1 * 9.81):
            omega_epsilon_acc = np.zeros((3, 1))
        else:
            omega_epsilon_acc = np.cross(a_b.T, (est_R @ a_i).T).T
            b_dot = -self.gains.Ki_acc * omega_epsilon_acc
            est_b = est_b + b_dot * self.dT

        omega_epsilon_mag = np.cross(h_b.T, (est_R @ h_i).T).T
        b_dot = -self.gains.Ki_mag * omega_epsilon_mag
        est_b = est_b + b_dot * self.dT

        est_w = omega - est_b
        temp = omega - est_b + self.gains.Kp_acc * omega_epsilon_acc + self.gains.Kp_mag * omega_epsilon_mag
        est_R = expm(-self.skew(temp) * self.dT) @ est_R

        return est_b, est_w.tolist(), est_R.tolist()
    
    def estimateCourse(self, sensorData=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0, accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0, gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0), estimatedState=States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0, dcm=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])):
        """
        Function to estimate the vehicle’s course (chi) using GPS. For the internal course rate, we are using dot{psi} rather than dot{chi} and letting the bias estimate clean it up.
        
        :param sensorData: The noisy sensors data to be filtered
        :param estimatedState: The previous estimate of the state
        :return: b_est, chi_est, the estimated course rate bias and estimated course
        """

        chi_est = estimatedState.chi
        b_x_prev = self.chi_bias
        b_x = 0

        chi_dot = 1 / math.cos(estimatedState.pitch) * (estimatedState.q * math.sin(estimatedState.roll) + estimatedState.r * math.cos(estimatedState.roll)) - b_x_prev # course rate estimate
        chi_est = chi_est + chi_dot * self.dT # integrate course rate

        if self.sensorsModel.tickCounter % (1 / self.sensorsModel.gpsUpdateHz) == self.sensorsModel.dT:
            b_dot_x = self.gains.Ki_chi * (sensorData.gps_cog - chi_est) # bias on course
            b_x = b_x_prev + b_dot_x * self.dT # integrate bias
            chi_dot = self.gains.Kp_chi * (sensorData.gps_cog - chi_est) - b_x # incorporate error  
            chi_est = chi_est + chi_dot * self.dT # complementary filter, new course update
            b_x_prev = b_x
        
        return b_x_prev, chi_est
    
    def getEstimatedState(self):
        """
        Wrapper function to return the estimated vehicle state.
        
        :return: State.VehicleState() object
        """

        return self.estimatedState
    
    def getEstimatorGains(self):
        """
        Wrapper function to return the vehicle estimator gains.
        
        :return: Controls.VehicleEstimatorGains() object
        """

        return self.gains
    
    def reset(self):
        """
        Wrapper function to reset the module, mainly the estimated state and biases. Does not reset the aerodynamics model, sensors model, or any of their dependents. Also does not reset the gains.
        
        :return: none
        """

        # self.estimatedState = States.vehicleState(pd=VPC.InitialDownPosition, u=VPC.InitialSpeed, dcm=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.estimatedState = States.vehicleState(pd=VPC.InitialDownPosition, dcm=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.baroFilter.reset()
        self.gyro_bias = [[0], [0], [0]]
        self.pitot_bias = 0
        self.b_gps_bias = 0
        self.b_dot = 0
    
    def setEstimatedState(self, estimatedState):
        """
        Wrapper function to set the estimated vehicle state.
        
        :param estimatedState: States.vehicleState() object
        :return: none
        """

        self.estimatedState = estimatedState
    
    def setEstimatorBiases(self, estimatedGyroBias=[[0], [0], [0]], estimatedPitotBias=0, estimatedChiBias=0, estimatedAscentRate=0, estimatedAltitudeGPSBias=0):
        """
        Wrapper to set the internal estimator biases.
        
        :param estimatedGyroBias: The estimator gyro biases for all 3 axes
        :param estimatedPitotBias: The estimated pitot bias
        :param estimatedChiBias: The estimated course bias
        :param estimatedAscentRate: The estimated ascent rate (alt_dot)
        :param estimatedAltitudeGPSBias: The estimated altitude bias from GPS
        :return: none
        """

        self.gyro_bias = estimatedGyroBias
        self.pitot_bias = estimatedPitotBias
        self.chi_bias = estimatedChiBias
        self.b_dot = estimatedAscentRate
        self.b_gps_bias = estimatedAltitudeGPSBias
    
    def setEstimatorGains(self, gains):
        """
        Wrapper function to set the vehicle estimator gains.
        
        :param gains: Controls.VehicleEstimatorGains() object
        :return: none
        """

        self.gains = gains
    
    # Helper Functions
    
    def skew(self, w):
        return np.matrix([[0, -w[2, 0], w[1, 0]],
                        [w[2, 0], 0, -w[0, 0]],
                        [-w[1, 0], w[0, 0], 0]])