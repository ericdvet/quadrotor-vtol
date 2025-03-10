"""
Author: Eric Vetha (evetha@ucsc.edu)
"""


import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel

class GaussMarkov:
    def __init__(self, dT=VPC.dT, tau=1e6, eta=0.0):
        """
        Function to initialize the GaussMarkov code that generates the exponentially correlated noise which is used for the slowly varying bias drift of the gyros as well as the GPS position. Implements the noise model characterized by a first order Gauss-Markov process: dv/dt = -1/tau * v + w, where w is a white noise process with N(0,eta).
        
        :param dT: time step [s]
        :param tau: correlation time [s]
        :param eta: standard deviation of white noise process
        :return: none
        """

        self.dT, self.tau, self.eta = dT, tau, eta
        self.v = 0.0

    def reset(self):
        """
        Wrapper function that resets the GaussMarkov model
        
        :return: none
        """

        self.v = 0.0

    def update(self, vnoise=None):
        """
        Function that updates the Gauss-Markov process, and returns the updated value (as well as updating the internal value that holds until the next call of the function).
        
        :param vnoise: optional parameter to drive the function with a known value. If None, then use random.gauss(0, eta)
        :return: v, new noise value (also updated internally)
        """
        
        if vnoise is None:
            self.v = self.v * math.exp(-self.dT / self.tau) + random.gauss(0, self.eta)
        else:
            self.v = self.v * math.exp(-self.dT / self.tau) + vnoise
        return self.v


class GaussMarkovXYZ:
    def __init__(self, dT=VPC.dT, tauX=1e6, etaX=0.0, tauY=None, etaY=None, tauZ=None, etaZ=None):
        """
        Function to aggregate three Gauss-Markov models into a triplet that returns the X, Y, and Z axes of the time-varying drift; if (tau, eta) are None, then will default to the same values for each model.
        
        :param dT: time step [s]
        :param tauX: correlation time for X [s]
        :param etaX: standard deviation of white noise process for X
        :param tauY: correlation time for Y [s]
        :param etaY: standard deviation of white noise process for Y
        :param tauZ: correlation time for Z [s]
        :param etaZ: standard deviation of white noise process for Z
        :return: none
        """

        self.dT = dT

        self.X = GaussMarkov(dT, tauX, etaX)
        
        # why does this not work when I check only tay for None...
        if tauY == None and etaY == None:
            self.Y = GaussMarkov(dT, tauX, etaX)
        else:
            self.Y = GaussMarkov(dT, tauY, etaY)
        
        if tauZ == None and etaZ == None:
            if tauY == None and etaY == None:
                self.Z = GaussMarkov(dT, tauX, etaX)
            else:
                self.Z = GaussMarkov(dT, tauY, etaY)
        else:
            self.Z = GaussMarkov(dT, tauZ, etaZ)

    def reset(self):
        """
        Wrapper function that resets the GaussMarkovXYZ models
        
        :return: none
        """

        self.X.reset()
        self.Y.reset()
        self.Z.reset()

    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):
        """
        Function that updates the Gauss-Markov processes, and returns the updated values (as well as updating the internal values that holds until the next call of the function).
        
        :param vXnoise: optional parameter to drive the X function with a known value. If None, then use random.gauss(0, etaX)
        :param vYnoise: optional parameter to drive the Y function with a known value. If None, then use random.gauss(0, etaY)
        :param vZnoise: optional parameter to drive the Z function with a known value. If None, then use random.gauss(0, etaZ)
        :return: vX, vY, vZ, new noise values for each axis (also updated internally)
        """

        # if vXnoise is None:
        #     vXnoise = random.gauss(0, self.etaX)
        # if vYnoise is None:
        #     vYnoise = random.gauss(0, self.etaY)
        # if vZnoise is None:
        #     vZnoise = random.gauss(0, self.etaZ)
        
        vX = self.X.update(vXnoise)
        vY = self.Y.update(vYnoise)
        vZ = self.Z.update(vZnoise)
        
        return vX, vY, vZ

class SensorsModel:
    def __init__(self, aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(), taugyro=VSC.gyro_tau, etagyro=VSC.gyro_eta, tauGPS=VSC.GPS_tau, etaGPSHorizontal=VSC.GPS_etaHorizontal, etaGPSVertical=VSC.GPS_etaVertical, gpsUpdateHz=VSC.GPS_rate):
        """
        Function to initialize the SensorsModel code. Will contain both the true sensors outputs and the noisy
        sensor outputs using the noise and biases found in the Constants.VehicleSensorConstants file. Biases for
        the sensors are set at instantiation and not updated further during the code run. All sensor outputs are in
        Engineering Units (it is assumed that the raw ADC counts to engineering unit scaling has already been
        done). Note that if the biases are set to None at instantiation, then they will be set to random values using
        uniform distribution with the appropriate bias scaling from the sensors constants. SensorsModel class
        keeps the Gauss-Markov models for gyro biases and GPS.
        
        :param aeroModel: handle to the VehicleAerodynamicsModel class
        :param taugyro: Gauss-Markov time constant for gyros [s]
        :param etagyro: Gauss-Markov process noise standard deviation [rad/s]
        :param tauGPS: Gauss-Markov time constant for GPS [s]
        :param etaGPSHorizontal: Gauss-Markov process noise standard deviation [m]
        :param etaGPSVertical: Gauss-Markov process noise standard deviation [m]
        :param gpsUpdateHz: Update rate for GPS measurements [Hz]
        :return: none
        """
        
        self.aeroModel = aeroModel
        self.taugyro, self.etagyro = taugyro, etagyro
        self.tauGPS, self.etaGPSHorizontal, self.etaGPSVertical = tauGPS, etaGPSHorizontal, etaGPSVertical

        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBiases = Sensors.vehicleSensors()
        self.sensorsSigmas = Sensors.vehicleSensors()
        self.sensorsNoisy = Sensors.vehicleSensors()

        self.sensorsBiases = self.initializeBiases()
        self.sensorsSigmas = self.initializeSigmas()

        self.gyroBiasModel = GaussMarkovXYZ(VPC.dT, taugyro, etagyro)
        self.gpsBiasModel = GaussMarkovXYZ(VPC.dT, tauGPS, etaGPSHorizontal, tauGPS, etaGPSHorizontal, tauGPS, etaGPSVertical)

        self.dT = self.aeroModel.getVehicleDynamicsModel().dT
        self.tickCounter = 0
        self.gpsUpdateHz = gpsUpdateHz

        self.Update = self.update

    def getSensorsNoisy(self):
        """
        Wrapper function to return the noisy sensor values
        
        :return: noisy sensor data, Sensors.vehicleSensors class
        """
        
        return self.sensorsNoisy

    def getSensorsTrue(self):
        """
        Wrapper function to return the true sensor values
        
        :return: true sensor values, Sensors.vehicleSensors class
        """
        
        return self.sensorsTrue

    def initializeBiases(self, gyroBias=VSC.gyro_bias, accelBias=VSC.accel_bias, magBias=VSC.mag_bias, baroBias=VSC.baro_bias, pitotBias=VSC.pitot_bias):
        """
        Function to generate the biases for each of the sensors. Biases are set with a uniform random number from -1 to 1 that is then multiplied by the sigma_bias.
        The biases for all sensors is returned as a Sensors.vehicleSensors class. Note that GPS is an unbiased sensor
        (though noisy), thus all the GPS biases are set to 0.0
        
        :param gyroBias: bias scaling for the gyros [rad/s]
        :param accelBias: bias scaling for the accelerometers [m/s^2]
        :param magBias: bias scaling for the magnetometers [nT]
        :param baroBias: bias scaling for the barometer [N/m^2]
        :param pitotBias: bias scaling for the pitot tube [N/m^2]
        :return: sensorBiases, class Sensors.vehicleSensors
        """
        
        bias = Sensors.vehicleSensors()

        bias.accel_x = random.uniform(-accelBias, accelBias)
        bias.accel_y = random.uniform(-accelBias, accelBias)
        bias.accel_z = random.uniform(-accelBias, accelBias)

        bias.gyro_x = random.uniform(-gyroBias, gyroBias)
        bias.gyro_y = random.uniform(-gyroBias, gyroBias)
        bias.gyro_z = random.uniform(-gyroBias, gyroBias)

        bias.baro = random.uniform(-baroBias, baroBias)
        
        bias.mag_x = random.uniform(-magBias, magBias)
        bias.mag_y = random.uniform(-magBias, magBias)
        bias.mag_z = random.uniform(-magBias, magBias)

        bias.pitot = random.uniform(-pitotBias, pitotBias)

        bias.gps_alt = 0.0
        bias.gps_cog = 0.0
        bias.gps_e = 0.0
        bias.gps_n = 0.0
        bias.gps_sog = 0.0

        return bias

    def initializeSigmas(self, gyroSigma=VSC.gyro_sigma, accelSigma=VSC.accel_sigma, magSigma=VSC.mag_sigma, baroSigma=VSC.baro_sigma, pitotSigma=VSC.pitot_sigma, gpsSigmaHorizontal=VSC.GPS_sigmaHorizontal, gpsSigmaVertical=VSC.GPS_sigmaVertical, gpsSigmaSOG=VSC.GPS_sigmaSOG, gpsSigmaCOG=VSC.GPS_sigmaCOG):
        """
        Function to gather all of the white noise standard deviations into a single vehicleSensor class object. These will be used as the input to
        generating the white noise added to each sensor when generating the noisy sensor data.
        
        :param gyroSigma: gyro white noise [rad/s]
        :param accelSigma: accelerometer white noise [m/s^2]
        :param magSigma: magnetometer white noise [nT]
        :param baroSigma: barometer white noise [N/m]
        :param pitotSigma: airspeed white noise [N/m]
        :param gpsSigmaHorizontal: GPS horizontal white noise [m]
        :param gpsSigmaVertical: GPS vertical white noise [m]
        :param gpsSigmaSOG: GPS Speed over ground white noise [m/s]
        :param gpsSigmaCOG: GPS Course over ground white noise, nominal [rad]
        :return: sensorSigmas, class Sensors.vehicleSensors
        """
        
        sigma = Sensors.vehicleSensors()

        sigma.accel_x = accelSigma
        sigma.accel_y = accelSigma
        sigma.accel_z = accelSigma

        sigma.gyro_x = gyroSigma
        sigma.gyro_y = gyroSigma
        sigma.gyro_z = gyroSigma

        sigma.baro = baroSigma

        sigma.mag_x = magSigma
        sigma.mag_y = magSigma
        sigma.mag_z = magSigma

        sigma.gps_alt = gpsSigmaVertical
        sigma.gps_cog = gpsSigmaCOG
        sigma.gps_e = gpsSigmaHorizontal
        sigma.gps_n = gpsSigmaHorizontal
        sigma.gps_sog = gpsSigmaSOG 

        sigma.pitot = pitotSigma

        return sigma


    def reset(self):
        """
        Function to reset the module to run again. Should reset the Gauss-Markov models, re-initialize the sensor
        biases, and reset the sensors true and noisy to pristine conditions
        
        :return: none
        """
        
        self.gyroBiasModel.reset()
        self.gpsBiasModel.reset()

        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsNoisy = Sensors.vehicleSensors()
        self.sensorsBiases = Sensors.vehicleSensors()
        self.sensorsSigmas = Sensors.vehicleSensors()

        self.sensorsBiases = self.initializeBiases()
        self.sensorsSigmas = self.initializeSigmas()


    def setSensorsNoisy(self, sensorsNoisy=Sensors.vehicleSensors()):
        """
        Wrapper function to set the noisy sensor values
        
        :param sensorsNoisy: Sensors.vehicleSensors object
        :return: none
        """
        
        self.sensorsNoisy = sensorsNoisy

    def setSensorsTrue(self, sensorsTrue=Sensors.vehicleSensors()):
        """
        Wrapper function to set the true sensor values
        
        :param sensorsTrue: Sensors.vehicleSensors object
        :return: none
        """
        
        self.sensorsTrue = sensorsTrue

    def update(self):
        """
        Wrapper function to update the Sensors (both true and noisy) using the state and dot held within the
        self.AeroModel. Note that we are passing in a handle to the class for this so we don’t have to explicitly
        use getters from the VehicleAerodynamics model. Sensors states are updated internally within self.
        
        :return: none
        """
        
        self.sensorsTrue = self.updateSensorsTrue(self.sensorsTrue, self.aeroModel.getVehicleState(), self.aeroModel.getVehicleDynamicsModel().getVehicleDerivative())
        self.sensorsNoisy = self.updateSensorsNoisy(self.sensorsTrue, self.sensorsNoisy, self.sensorsBiases, self.sensorsSigmas)
        self.tickCounter += self.dT

    def updateAccelsTrue(self, state, dot):
        """
        Function to update the accelerometer sensor. Will be called within the updateSensors functions.
        
        :param state: class States.vehicleState, current vehicle state
        :param dot: class States.vehicleState, current state derivative
        :return: accel_x, accel_y, accel_z, body frame specific force [m/s^2]
        """
        
        accel_x = dot.u + state.q * state.w - state.r * state.v + VPC.g0 * math.sin(state.pitch)
        accel_y = dot.v + state.r * state.u - state.p * state.w - VPC.g0 * math.cos(state.pitch) * math.sin(state.roll)
        accel_z = dot.w + state.p * state.v - state.q * state.u - VPC.g0 * math.cos(state.pitch) * math.cos(state.roll)

        return accel_x, accel_y, accel_z

    def updateGPSTrue(self, state, dot):
        """
        Function to update the GPS sensor state (this will be called to update the GPS data from the state and the derivative) at the required rate. Note that GPS reports back altitude as + above mean sea level.
        
        :param state: class States.vehicleState, current vehicle state
        :param dot: class States.vehicleState, current state derivative
        :return: gps_n, [North- m], gps_e [East- m], gps_alt [Altitude- m], gps_SOG [Speed over ground, m/s], gps_COG [Course over ground, rad]
        """
        
        gps_n = state.pn
        gps_e = state.pe
        gps_alt = -state.pd

        gps_sog = math.hypot(dot.pn, dot.pe)
        gps_cog = math.atan2(dot.pe, dot.pn)

        return gps_n, gps_e, gps_alt, gps_sog, gps_cog

    def updateGyrosTrue(self, state):
        """
        Function to update the rate gyro sensor. Will be called within the updateSensors functions.
        
        :param state: class States.vehicleState, current vehicle state
        :return: gyro_x, gyro_y, gyro_z, body frame rotation rates [rad/s]
        """

        return state.p, state.q, state.r

    def updateMagsTrue(self, state):
        """
        Function to update the magnetometer sensor. Will be called within the updateSensors functions.
        
        :param state: class States.vehicleState, current vehicle state
        :return: mag_x, mag_y, mag_z, body frame magnetic field [nT]
        """
        
        mag = MatrixMath.multiply(state.R, VSC.magfield)

        return mag[0][0], mag[1][0], mag[2][0]

    def updatePressureSensorsTrue(self, state):
        """
        Function to update the pressure sensors onboard the aircraft. Will be called within the updateSensors functions. The two pressure sensors are static pressure (barometer) and dynamic pressure (pitot tube). The barometric pressure is references off of the ground static pressure in VehicleSensorConstants at Pground.
        
        :param state: class States.vehicleState, current vehicle state
        :return: baro, pitot in [N/m^2]
        """
        
        baro = VSC.Pground - VPC.rho * VPC.g0 * (- state.pd)
        pitot = 0.5 * VPC.rho * state.Va ** 2

        return baro, pitot

    def updateSensorsNoisy(self, trueSensors=Sensors.vehicleSensors(), noisySensors=Sensors.vehicleSensors(), sensorBiases=Sensors.vehicleSensors(), sensorSigmas=Sensors.vehicleSensors()):
        """
        Function to generate the noisy sensor data given the true sensor readings, the biases, and the sigmas for the white noise on each sensor. The gauss markov models for the gyro biases and GPS positions are updated here. The GPS COG white noise must be scaled by the ratio of VPC.initialSpeed / actual ground speed. GPS is only updated if the correct number of ticks have gone by to indicate that a new GPS measurement should be generated. The GPS COG must be limited to within +/- PI. If no GPS update has occurred, then the values for the GPS sensors should be copied from the noisySensors input to the output.
        
        :param trueSensors: Sensors.vehicleSensors class, true values (no noise)
        :param noisySensors: Sensors.vehicleSensors class, previous noisy sensor values
        :param sensorBiases: Sensors.vehicleSensors class, fixed biases for each sensor
        :param sensorSigmas: Sensors.vehicleSensors class, standard deviations of white noise on each sensor
        :return: noisy sensor data, Sensors.vehicleSensors class.
        """

        piplineTest = False

        sensors = Sensors.vehicleSensors()
        
        if piplineTest:
            return sensors
        else:
            sensors.accel_x = trueSensors.accel_x + random.gauss(0, sensorSigmas.accel_x) + sensorBiases.accel_x
            sensors.accel_y = trueSensors.accel_y + random.gauss(0, sensorSigmas.accel_y) + sensorBiases.accel_y
            sensors.accel_z = trueSensors.accel_z + random.gauss(0, sensorSigmas.accel_z) + sensorBiases.accel_z

            sensors.gyro_x = trueSensors.gyro_x + sensorBiases.gyro_x + self.gyroBiasModel.X.update()
            sensors.gyro_y = trueSensors.gyro_y + sensorBiases.gyro_y + self.gyroBiasModel.Y.update()
            sensors.gyro_z = trueSensors.gyro_z + sensorBiases.gyro_z + self.gyroBiasModel.Z.update()

            sensors.mag_x = trueSensors.mag_x + random.gauss(0, sensorSigmas.mag_x) + sensorBiases.mag_x
            sensors.mag_y = trueSensors.mag_y + random.gauss(0, sensorSigmas.mag_y) + sensorBiases.mag_y
            sensors.mag_z = trueSensors.mag_z + random.gauss(0, sensorSigmas.mag_z) + sensorBiases.mag_z

            sensors.baro = trueSensors.baro + random.gauss(0, sensorSigmas.baro) + sensorBiases.baro

            sensors.pitot = trueSensors.pitot + random.gauss(0, sensorSigmas.pitot) + sensorBiases.pitot

            if self.tickCounter % (1 / self.gpsUpdateHz) == 0:
                sensors.gps_n = trueSensors.gps_n + random.gauss(0, sensorSigmas.gps_n) + self.gpsBiasModel.X.update()
                sensors.gps_e = trueSensors.gps_e + random.gauss(0, sensorSigmas.gps_e) + self.gpsBiasModel.Y.update()
                sensors.gps_alt = trueSensors.gps_alt + random.gauss(0, sensorSigmas.gps_alt) + self.gpsBiasModel.Z.update()
                sensors.gps_sog = trueSensors.gps_sog + random.gauss(0, sensorSigmas.gps_sog) 
                sensors.gps_cog = trueSensors.gps_cog + random.gauss(0, sensorSigmas.gps_cog)
            else:
                sensors.gps_n, sensors.gps_e, sensors.gps_alt = noisySensors.gps_n, noisySensors.gps_e, noisySensors.gps_alt
                sensors.gps_sog, sensors.gps_cog = noisySensors.gps_sog, noisySensors.gps_cog
            
            return sensors

    def updateSensorsTrue(self, prevTrueSensors, state, dot):
        """
        Function to generate the true sensors given the current state and state derivative. Sensor suite is 3-axis accelerometer, 3-axis rate gyros, 3-axis magnetometers, a barometric altimeter, a pitot airspeed, and GPS with an update rate specified in the VehicleSensorConstants file. For the GPS update, the previous value is returned until a new update occurs. Previous value is contained within prevTrueSensors.
        
        :param prevTrueSensors: class Sensors.vehicleSensors(), previous true sensor readings (no noise)
        :param state: class States.vehicleState, current vehicle state
        :param dot: class States.vehicleState, current state derivative
        :return: true sensor outputs (no noise, no biases), Sensors.vehicleSensors class
        """
        
        sensors = Sensors.vehicleSensors()

        sensors.accel_x, sensors.accel_y, sensors.accel_z = self.updateAccelsTrue(state, dot)
        sensors.gyro_x, sensors.gyro_y, sensors.gyro_z = self.updateGyrosTrue(state)
        sensors.mag_x, sensors.mag_y, sensors.mag_z = self.updateMagsTrue(state)
        sensors.baro, sensors.pitot = self.updatePressureSensorsTrue(state)

        if self.tickCounter % (1 / self.gpsUpdateHz) == 0:
            sensors.gps_n, sensors.gps_e, sensors.gps_alt, sensors.gps_sog, sensors.gps_cog = self.updateGPSTrue(state, dot)
        else:
            sensors.gps_n = prevTrueSensors.gps_n
            sensors.gps_e = prevTrueSensors.gps_e
            sensors.gps_alt = prevTrueSensors.gps_alt
            sensors.gps_sog = prevTrueSensors.gps_sog
            sensors.gps_cog = prevTrueSensors.gps_cog

        return sensors