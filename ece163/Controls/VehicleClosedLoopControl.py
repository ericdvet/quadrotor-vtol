"""
Author: Eric Vetha (evetha@ucsc.edu)

VehicleClosedLoopControl: PDControl, PIControl, PIDControl

File that implements the closed loop control using the successive loop closure structure described in Beard Chapter
6. In addition to implementation of the control loops, the file includes classes for PD control (where the derivative
is available as a direct input), PI control with anti-windup and a reset, and a PID loop with anti-windup (where the
derivative is again available directly).
"""

import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule
import ece163.Controls.VehicleEstimator as VehicleEstimator
import ece163.Sensors.SensorsModel as SensorsModel

class PDControl:
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Functions which implement the PD control with saturation where the derivative is available as a separate
        input to the function. The output is: u = u_ref + Kp * error- Kd * dot{error} limited between lowLimit
        and highLimit.
        
        :param kp: Proportional gain
        :param kd: Derivative gain
        :param trim: Trim output (added to the loop computed output)
        :param lowLimit: Lower limit to saturate control
        :param highLimit: Upper limit to saturate control
        """

        self.kp, self.kd = kp, kd
        self.trim = trim
        self.lowLimit, self.highLimit = lowLimit, highLimit
        
    def Update(self, command=0.0, current=0.0, derivative=0.0):
        """
        Calculates the output of the PD loop given the gains and limits from instantiation, and using the command,
        actual, and derivative inputs. Output is limited to between lowLimit and highLimit from instantiation.
        
        :param command: Reference command
        :param current: Actual output (or sensor)
        :param derivative: Derivative of the output or sensor
        :return: u [control] limited to saturation bounds
        """

        u = self.trim + self.kp * (command - current) - self.kd * derivative
        if u > self.highLimit:
            return self.highLimit
        elif u < self.lowLimit:
            return self.lowLimit
        else:
            return u

    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Function to set the gains for the PD control block (including the trim output and the limits)
        
        :param kp: Proportional gain
        :param kd: Derivative gain
        :param trim: Trim output (added to the loop computed output)
        :param lowLimit: Lower limit to saturate control
        :param highLimit: Upper limit to saturate control
        """

        self.kp, self.kd = kp, kd
        self.trim = trim
        self.lowLimit, self.highLimit = lowLimit, highLimit

class PIControl:
    def __init__(self, dT=0.01, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Functions which implement the PI control with saturation where the integrator has both a reset and an anti
        windup such that when output saturates, the integration is undone and the output forced the output to the
        limit. The output is: u = u_ref + Kp * error + Ki*integral{error} limited between lowLimit and highLimit.
        
        :param dT: Time step [s], required for integration
        :param kp: Proportional gain
        :param ki: Integral gain
        :param trim: Trim input
        :param lowLimit: Low saturation limit
        :param highLimit: High saturation limit
        """

        self.kp, self.ki = kp, ki
        self.trim = trim
        self.lowLimit, self.highLimit = lowLimit, highLimit
        self.dT = dT
        self.accumulator = 0.0
        self.prevError = 0.0
        
    def Update(self, command=0.0, current=0.0):
        """
        Calculates the output of the PI loop given the gains and limits from instantiation, and using the command
        and current or actual inputs. Output is limited to between lowLimit and highLimit from instantiation.
        Integration for the integral state is done using trapezoidal integration, and anti-windup is implemented
        such that if the output is out of limits, the integral state is not updated (no additional error accumulation).
        
        :param command: Reference command
        :param current: Current output or sensor
        :return: u [output] limited to saturation bounds
        """

        temp_accumulator = self.accumulator + self.dT * ((command - current) + self.prevError) / 2.0
        u = self.trim + self.kp * (command - current) + self.ki * temp_accumulator
        if u > self.highLimit:
            self.prevError = command - current
            return self.highLimit
        elif u < self.lowLimit:
            self.prevError = command - current
            return self.lowLimit
        else:
            self.accumulator = temp_accumulator
            self.prevError = command - current
            return u

    def resetIntegrator(self):
        """
        Function to reset the integration state to zero, used when switching modes or otherwise resetting the integral
        state.
        """

        self.accumulator = 0.0

    def setPIGains(self, dT=0.01, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Function to set the gains for the PI control block (including the trim output and the limits)
        """

        self.kp, self.ki = kp, ki
        self.trim = trim
        self.lowLimit, self.highLimit = lowLimit, highLimit

class PIDControl:
    def __init__(self, dT=0.01, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Functions which implement the PID control with saturation where the integrator has both a reset and an
        anti-windup such that when output saturates, the integration is undone and the output forced the output
        to the limit. Function assumes that physical derivative is available (e.g.: roll and p), not a numerically
        derived one. The output is: u = u_ref + Kp * error- Kd * dot{error} + Ki * integral{error} limited between
        lowLimit and highLimit.
        
        :param dT: Time step [s], required for integration
        :param kp: Proportional gain
        :param kd: Derivative gain
        :param ki: Integral gain
        :param trim: Trim input
        :param lowLimit: Low saturation limit
        :param highLimit: High saturation limit
        """

        self.kp, self.kd, self.ki = kp, kd, ki
        self.trim = trim
        self.lowLimit, self.highLimit = lowLimit, highLimit
        self.dT = dT
        self.accumulator = 0.0
        self.prevError = 0.0
        
    def Update(self, command=0.0, current=0.0, derivative=0.0):
        """
        Calculates the output of the PI loop given the gains and limits from instantiation, and using the command
        and current or actual inputs. Output is limited to between lowLimit and highLimit from instantiation.
        Integration for the integral state is done using trapezoidal integration, and anti-windup is implemented
        such that if the output is out of limits, the integral state is not updated (no additional error accumulation).
        
        :param command: Reference command
        :param current: Current output or sensor
        :return: u [output] limited to saturation bounds
        """

        temp_accumulator = self.accumulator + self.dT * ((command - current) + self.prevError) / 2.0
        u = self.trim + self.kp * (command - current) - self.kd * derivative + self.ki * temp_accumulator
        if u > self.highLimit:
            self.prevError = command - current
            return self.highLimit
        elif u < self.lowLimit:
            self.prevError = command - current
            return self.lowLimit
        else:
            self.accumulator = temp_accumulator
            self.prevError = command - current
            return u

    def resetIntegrator(self):
        """
        Function to reset the integration state to zero, used when switching modes or otherwise resetting the integral
        state.
        """

        self.accumulator = 0.0

    def setPIDGains(self, dT=0.01, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """
        Function to set the gains for the PID control block (including the trim output and the limits)

        :param dT: Time step [s], required for integration
        :param kp: Proportional gain
        :param kd: Derivative gain
        :param ki: Integral gain
        :param trim: Trim input
        :param lowLimit: Low saturation limit
        :param highLimit: High saturation limit
        :return: None
        """

        self.kp, self.kd, self.ki = kp, kd, ki
        self.trim = trim
        self.lowLimit, self.highLimit = lowLimit, highLimit

class VehicleClosedLoopControl:
    """
    Class that implements the entire closed loop control using the successive loop closure method of Beard
    Chapter 6. The class contains everything from control loops (PD or PI loops) along with the gains, which
    when given the reference commands will compute the control surfaces, which will then (along with the
    state) dictate the forces and moments, which will them compute the next state. Contains the required state
    for the altitude hold state machine from the enumeration class in Controls.AltitudeStates
    If the sensor model is active, self gets a SensorsModel() subclass initialized here (Lab 5 / Chapter 7). If the
    estimator is active, self gets a VehicleEstimator() subclass initialized here (Lab 6 / Chapter 8).
    
    :param dT: time step [s], defaults to VehiclePhysicalParameters.dT
    :param rudderControlSource: Either “SIDESLIP” or “YAW”. Determines whether the rudder
    is controlled by sideslip angle (as in Beard) or by yaw angle.
    :param useSensors: Either True or False. Flag to run the sensors model (update and reset).
    :param useEstimator: Either True or False. Flag to run the vehicle estimator (update and reset)
    and to use the estimated state instead of the true state to control the UAV.
    :return: none
    """
    
    def __init__(self, dT=VPC.dT, rudderControlSource="SIDESLIP", useSensors=False, useEstimator=False):
        """
        Initializes the VehicleClosedLoopControl class with the given parameters.
        
        :param dT: time step [s], defaults to VehiclePhysicalParameters.dT
        :param rudderControlSource: Either “SIDESLIP” or “YAW”. Determines whether the rudder
        is controlled by sideslip angle (as in Beard) or by yaw angle.
        :param useSensors: Either True or False. Flag to run the sensors model (update and reset).
        :param useEstimator: Either True or False. Flag to run the vehicle estimator (update and reset)
        and to use the estimated state instead of the true state to control the UAV.
        :return: none
        """

        self.update = self.Update

        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        self.controlGains = Controls.controlGains()
        self.trimInputs = Inputs.controlInputs()
        self.outputControls = Inputs.controlInputs()
        self.dT = dT
        self.mode = Controls.AltitudeStates.HOLDING

        self.rollFromCourse = PIControl(dT=dT)
        self.rudderFromSideslip = PIControl(dT=dT)
        self.throttleFromAirspeed = PIControl(dT=dT)
        self.pitchFromAltitude = PIControl(dT=dT)
        self.pitchFromAirspeed = PIControl(dT=dT)
        self.elevatorFromPitch = PDControl()
        self.aileronFromRoll = PIDControl(dT=dT)

        self.useSensors = useSensors
        if self.useSensors:
            self.SensorsModel = SensorsModel.SensorsModel(aeroModel=self.VAM)
        
        self.useEstimator = useEstimator
        if self.useEstimator:
            self.VehicleEstimator = VehicleEstimator.VehicleEstimator(dT=self.dT, sensorsModel=self.SensorsModel)
    
    def UpdateControlCommands(self, referenceCommands, state):
        """
        Function that implements the full closed loop controls using the commanded inputs of airspeed, altitude,
        and course (chi). Calls all of the submodules below it to implement the full flight dynamics under closed
        loop control. Note that the internal commandedPitch and commandedRoll of the reference commands are
        altered by this function, and this is used to track the reference commands by the simulator. Note that if
        running the estimator, then the state used by this function is the estimated state.
        You will need to add or subtract 360 degrees (2*pi) from your internal course (chi) when the error is outside
        of +/- 180 degrees (pi radians). There is never a course error of more than 180 degrees, so if you do see
        such an error, it is because signed angles and reciprocal headings (e.g.:-180 and +180 are pointed in the
        same direction).
        
        :param referenceCommands: high level autopilot commands (note: altered by function)
        :param state: vehicle state (either actual or estimated)
        :return: controlSurfaceOutputs: vehicle control inputs from class Inputs->controlInputs
        """

        upper_thresh = referenceCommands.commandedAltitude + VPC.altitudeHoldZone
        lower_thresh = referenceCommands.commandedAltitude - VPC.altitudeHoldZone
        altitude = -state.pd

        if referenceCommands.commandedCourse - state.chi <= -math.pi:
            state.chi = state.chi - 2 * math.pi
        elif referenceCommands.commandedCourse - state.chi >= math.pi:
            state.chi = state.chi + 2 * math.pi

        if self.mode == Controls.AltitudeStates.DESCENDING:
            if lower_thresh < altitude < upper_thresh:
                if self.mode != Controls.AltitudeStates.HOLDING:
                    self.pitchFromAltitude.resetIntegrator()
                self.mode = Controls.AltitudeStates.HOLDING
        elif self.mode == Controls.AltitudeStates.HOLDING:
            if altitude < lower_thresh:
                if self.mode != Controls.AltitudeStates.CLIMBING:
                    self.pitchFromAirspeed.resetIntegrator()
                self.mode = Controls.AltitudeStates.CLIMBING
            elif altitude > upper_thresh:
                if self.mode != Controls.AltitudeStates.DESCENDING:
                    self.pitchFromAirspeed.resetIntegrator()
                self.mode = Controls.AltitudeStates.DESCENDING
        elif self.mode == Controls.AltitudeStates.CLIMBING:
            if lower_thresh < altitude < upper_thresh:
                if self.mode != Controls.AltitudeStates.HOLDING:
                    self.pitchFromAltitude.resetIntegrator()
                self.mode = Controls.AltitudeStates.HOLDING
        else:
            if self.mode != Controls.AltitudeStates.HOLDING:
                self.pitchFromAltitude.resetIntegrator()
            self.mode = Controls.AltitudeStates.HOLDING
        
        if self.mode == Controls.AltitudeStates.DESCENDING:
            throttleCommand = VPC.minControls.Throttle
            pitchCommand = self.pitchFromAirspeed.Update(command=referenceCommands.commandedAirspeed, current=state.Va)
        elif self.mode == Controls.AltitudeStates.HOLDING:
            throttleCommand = self.throttleFromAirspeed.Update(command=referenceCommands.commandedAirspeed, current=state.Va)
            pitchCommand = self.pitchFromAltitude.Update(command=referenceCommands.commandedAltitude, current=-state.pd)
        elif self.mode == Controls.AltitudeStates.CLIMBING:
            throttleCommand = VPC.maxControls.Throttle
            pitchCommand = self.pitchFromAirspeed.Update(command=referenceCommands.commandedAirspeed, current=state.Va)

        rollCommand = self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi)
        aileronCommand = self.aileronFromRoll.Update(command=rollCommand, current=state.roll, derivative=state.p)

        referenceCommands.commandedRoll = rollCommand
        referenceCommands.commandedPitch = pitchCommand

        controlInputs = Inputs.controlInputs()
        controlInputs.Throttle = throttleCommand
        controlInputs.Aileron = aileronCommand
        controlInputs.Elevator = self.elevatorFromPitch.Update(command=pitchCommand, current=state.pitch, derivative=state.q)
        controlInputs.Rudder = self.rudderFromSideslip.Update(0.0, state.beta)

        return controlInputs
    
    def getControlGains(self):
        """
        Wrapper function to extract control gains from the class.
        
        :return: controlGains, class from Controls.controlGains
        """

        return self.controlGains
    
    def getSensorsModel(self):
        """
        Wrapper function to extract the internal SensorsModel in order to access the various functions that are
        associated with the Sensors model (such as setting and getting the sensor measurements and parameters).
        
        :return: SensorsModel, class from SensorsModel
        """

        return self.SensorsModel
    
    def getTrimInputs(self):
        """
        Wrapper function to get the trim inputs from the class.
        
        :return: trimInputs, class from Inputs.controlInputs (Throttle, Elevator, Aileron, Rudder)
        """

        return self.trimInputs
    
    def getVehicleAerodynamicsModel(self):
        """
        Wrapper function to extract the internal VehicleAerodynamicsModel in order to access the various functions
        that are associated with the Aero model (such as setting and getting the wind state and model).
        
        :return: VehicleAerodynamicsModel, class from VehicleAerodynamicsModel
        """

        return self.VAM
    
    def getVehicleControlSurfaces(self):
        """
        Wrapper function to extract control outputs (Throttle, Aileron, Elevator, Rudder) from the class.
        
        :return: controlInputs, class from Inputs.controlInputs
        """

        return self.outputControls
    
    def getVehicleEstimator(self):
        """
        Wrapper function to extract the internal VehicleEstimator in order to access the various functions that are
        associated with the vehicle estimator (such as setting and getting the estimated state and gains).
        
        :return: VehicleEstimator, class from VehicleEstimator
        """
    
        return self.VehicleEstimator
    
    def getVehicleState(self):
        """
        Wrapper function to extract vehicle state from the class.
        
        :return: vehicleState, class from States.vehicleState
        """

        return self.VAM.getVehicleState()
    
    def reset(self):
        """
        Resets the module to run again. Does not overwrite control gains, but does reset the integral states of all of
        the PI control loops.
        If the sensor model is active, also runs SensorsModel.reset() here (Lab 5 / Chapter 7). If the estimator is
        active, also runs VehicleEstimator.reset() here (Lab 6 / Chapter 8).
        
        :return: none
        """

        self.rollFromCourse.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()
        self.aileronFromRoll.resetIntegrator()

        self.VAM.reset()

        if self.useSensors:
            self.SensorsModel.reset()

        if self.useEstimator:
            self.VehicleEstimator.reset()
    
    def setControlGains(self, controlGains=Controls.controlGains()):
        """
        Function to set all of the gains from the controlGains previously computed to the correct places within the
        various control loops to do the successive loop closure (see Beard Chapter 6). Control loop limits are taken
        from the VehiclePhysicalConstants file, trim inputs are taken from self.trimInputs.
        
        :param controlGains: controlGains class, has the PID loop gains for each loop
        :return: none, values are updated in self.
        """
        
        kp_roll, ki_roll, kd_roll = controlGains.kp_roll, controlGains.ki_roll, controlGains.kd_roll
        kp_sideslip, ki_sideslip = controlGains.kp_sideslip, controlGains.ki_sideslip
        kp_course, ki_course = controlGains.kp_course, controlGains.ki_course
        kp_pitch, kd_pitch = controlGains.kp_pitch, controlGains.kd_pitch
        kp_altitude, ki_altitude = controlGains.kp_altitude, controlGains.ki_altitude
        kp_SpeedFromThrottle, ki_SpeedFromThrottle = controlGains.kp_SpeedfromThrottle, controlGains.ki_SpeedfromThrottle
        kp_SpeedFromElevator, ki_SpeedFromElevator = controlGains.kp_SpeedfromElevator, controlGains.ki_SpeedfromElevator

        bankAngleLimit = math.radians(VPC.bankAngleLimit)
        pitchAngleLimit = math.radians(VPC.pitchAngleLimit)
        minThrottle, minAileron, minElevator, minRudder = VPC.minControls.Throttle, VPC.minControls.Aileron, VPC.minControls.Elevator, VPC.minControls.Rudder
        maxThrottle, maxAileron, maxElevator, maxRudder = VPC.maxControls.Throttle, VPC.maxControls.Aileron, VPC.maxControls.Elevator, VPC.maxControls.Rudder

        trimThrottle, trimAileron, trimElevator, trimRudder = self.trimInputs.Throttle, self.trimInputs.Aileron, self.trimInputs.Elevator, self.trimInputs.Rudder

        self.rollFromCourse.setPIGains(kp=kp_course, ki=ki_course, trim=0.0, lowLimit=-bankAngleLimit, highLimit=bankAngleLimit)
        self.rudderFromSideslip.setPIGains(kp=kp_sideslip, ki=ki_sideslip, trim=trimRudder, lowLimit=minRudder, highLimit=maxRudder)
        self.throttleFromAirspeed.setPIGains(kp=kp_SpeedFromThrottle, ki=ki_SpeedFromThrottle, trim=trimThrottle, lowLimit=minThrottle, highLimit=maxThrottle)
        self.pitchFromAltitude.setPIGains(kp=kp_altitude, ki=ki_altitude, trim=0.0, lowLimit=-pitchAngleLimit, highLimit=pitchAngleLimit)
        self.pitchFromAirspeed.setPIGains(kp=kp_SpeedFromElevator, ki=ki_SpeedFromElevator, trim=0.0, lowLimit=-pitchAngleLimit, highLimit=pitchAngleLimit)
        self.elevatorFromPitch.setPDGains(kp=kp_pitch, kd=kd_pitch, trim=trimElevator, lowLimit=minElevator, highLimit=maxElevator)
        self.aileronFromRoll.setPIDGains(kp=kp_roll, kd=kd_roll, ki=ki_roll, trim=trimAileron, lowLimit=minAileron, highLimit=maxAileron)

        self.controlGains = controlGains

    
    def setTrimInputs(self, trimInputs=Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):
        """
        Wrapper function to inject the trim inputs into the class.
        
        :param trimInputs: from Inputs.controlInputs (Throttle, Elevator, Aileron, Rudder)
        :return: none
        """

        self.trimInputs = trimInputs
    
    def setVehicleState(self, state):
        """
        Wrapper function to inject vehicle state into the class.
        
        :param state: from States.vehicleState class
        :return: none
        """

        self.VAM.setVehicleState(state)
    
    def Update(self, referenceCommands=Controls.referenceCommands()):
        """
        Update(self, referenceCommands=Controls.referenceCommands) Function that wraps the UpdateControl
        Commands and feeds it the correct state (estimated or otherwise) to generate the controlSurfaceOutputs.
        Updates the true vehicle state internally using the vehicleAerodynamics.Update command.
        If the sensor model is active, updates the sensors using SensorsModel.Update command (Lab 5 / Chapter
        7). If the estimator is active, it also updates the estimated state using VehicleEstimator.Update command
        (Lab 6 / Chapter 8).
        
        :param referenceCommands: reference mid-level autopilot commands from class Controls.referenceCommands()
        """

        self.outputControls = self.UpdateControlCommands(referenceCommands, self.VAM.getVehicleState())
        self.VAM.Update(self.outputControls)

        if self.useSensors:
            self.SensorsModel.update()
        
        if self.useEstimator:
            self.VehicleEstimator.update()
    
    # def update(self, referenceCommands=Controls.referenceCommands()):
    #     """
    #     Update(self, referenceCommands=Controls.referenceCommands) Function that wraps the UpdateControl
    #     Commands and feeds it the correct state (estimated or otherwise) to generate the controlSurfaceOutputs.
    #     Updates the true vehicle state internally using the vehicleAerodynamics.Update command.
    #     If the sensor model is active, updates the sensors using SensorsModel.Update command (Lab 5 / Chapter
    #     7). If the estimator is active, it also updates the estimated state using VehicleEstimator.Update command
    #     (Lab 6 / Chapter 8).
        
    #     :param referenceCommands: reference mid-level autopilot commands from class Controls.referenceCommands()
    #     """

    #     self.outputControls = self.UpdateControlCommands(referenceCommands, self.VAM.getVehicleState())
    #     self.VAM.Update(self.outputControls)