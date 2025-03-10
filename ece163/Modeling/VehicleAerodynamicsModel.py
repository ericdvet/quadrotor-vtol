"""
Author: Eric Vetha (evetha@ucsc.edu)

Vehicle Aerodynamics Model will take the parameters from the VehiclePhysicalConstants and use the vehicle state to 
compute the forces and moments about the vehicle for integration. These are linearized aerodynamics using stability 
derivatives and simplified aerodynamics; the only deviation from this is the post-stall lift model used to convert lift 
from the linear model to a flat plate at an angle.
"""

import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC
import numpy as np

class VehicleAerodynamicsModel:

    def __init__(self, initialSpeed=25.0, initialHeight=-100.0):
        """
        Initialization of the internal classes which are used to track the vehicle aerodynamics and dynamics.
        
        :param initialSpeed: Defaults to VPC.InitialSpeed.
        :param initialHeight: Defaults to VPC.InitialDownPosition.
        """

        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.VDM = VehicleDynamicsModel.VehicleDynamicsModel()
        self.state = States.vehicleState(pd=initialHeight, u=initialSpeed)
        self.windModel = WindModel.WindModel()
        self.VDM.state = self.state
    
    def CalculateAirspeed(self, state, wind):
        """
        Calculates the total airspeed, as well as angle of attack and side-slip angles from the wind and current state.
        Needed for further aerodynamic force calculations. Va, wind speed [m/s], alpha, angle of attack [rad], and
        beta, side-slip angle [rad] are returned from the function. The state must be updated outside this function.
        
        Note: when total wind speed (math.hypot(wind.Wn, wind.We, wind.Wd)) is zero, set gamma wind to 0
        (otherwise it is undefined because arcsin is greater than 1.), Also, check is the total airspeed (Va) is zero
        before calculating the sideslip angle beta, if so, set beta to 0.0

        :param state: current vehicle state (need the velocities)
        :param wind: current wind state (global and gust)
        :return: Va, wind speed [m/s], alpha, angle of attack [rad], and beta, side-slip angle [rad]
        """

        W_s = math.hypot(wind.Wn, wind.We, wind.Wd)
        X_w = math.atan2(wind.We, wind.Wn)
        if W_s != 0:
            Y_w = -math.asin(wind.Wd / W_s)
        else:
            Y_w = 0
        R = np.matrix([
            [np.cos(X_w) * np.cos(Y_w), -np.sin(X_w), np.cos(X_w) * np.sin(Y_w)],
            [np.sin(X_w) * np.cos(Y_w), np.cos(X_w), np.sin(X_w) * np.sin(Y_w)],
            [-np.sin(Y_w), 0, np.cos(Y_w)]
        ])
        W_b = np.matrix(state.R) @ (
            np.matrix([[wind.Wn], [wind.We], [wind.Wd]]) + R @ np.matrix([[wind.Wu], [wind.Wv], [wind.Ww]])
        )
        w_u, w_v, w_w = W_b[0,0], W_b[1,0], W_b[2,0]

        u = state.u - w_u
        v = state.v - w_v
        w = state.w - w_w
        Va = math.sqrt(u ** 2 + v ** 2 + w ** 2)
        alpha = math.atan2(w, u)
        if Va != 0:
            beta = math.asin(v / Va)
        else:
            beta = 0.0
        return Va, alpha, beta

    def CalculateCoeff_alpha(self, alpha):
        """
        Function to calculate the Coefficient of Lift and Drag (and Moment) as a function of angle of attack. 
        Angle of attack (alpha) in [rad] is contained within the state.alpha and updated within the CalculateAirspeed 
        function. For pre-stall lift and drag, use simple linear model for lift: CL = CL0 + CLalpha * alpha, 
        and for the pre-stall drag use the parabolic form: CD = CDp + (CL(alpha))^2/(pi*AR*e), where CL(alpha) 
        is the pre-stall lift above.
        
        For post-stall, use the flat-plate models for lift and drag: CL = 2 * sin(alpha) * cos(alpha) and CD = 2 
        sin^2(alpha). Use the exponential blending function (sigma) outlined in the book to blend pre- and post-stall 
        lift and drag, with parameters of M and alpha0 taken from VehiclePhysicalParameters.py.
        
        Note: For CM use the same model throughout: CM = CM0 + CMalpha * alpha.
        
        :param alpha: Angle of Attack [rad].
        :return: Coefficient of Lift, CL_alpha (unitless), Coefficient of Drag, CD_alpha (unitless), Coefficient of Moment, CM_alpha (unitless).
        """

        sigma = (
            1 + np.exp(-VPC.M * (alpha - VPC.alpha0)) + np.exp(VPC.M * (alpha + VPC.alpha0))
            ) / (
                (1 + np.exp(-VPC.M * (alpha - VPC.alpha0))) * (1 + np.exp(VPC.M * (alpha + VPC.alpha0)))
                )
        C_L_attached = VPC.CL0 + VPC.CLalpha * alpha
        C_D_attached = VPC.CDp + (VPC.CL0 + VPC.CLalpha * alpha) ** 2 / (math.pi * VPC.AR * VPC.e)
        C_L_separated = 2 * math.sin(alpha) * math.cos(alpha)
        C_D_separated = 2 * math.sin(alpha) ** 2
        C_L = (1 - sigma) * C_L_attached + sigma * C_L_separated
        C_D = (1 - sigma) * C_D_attached + sigma * C_D_separated
        C_M = VPC.CM0 + VPC.CMalpha * alpha

        return C_L, C_D, C_M

    def CalculatePropForces(self, Va, Throttle):
        """
        Function to calculate the propeller forces and torques on the aircraft. Uses the fancy propeller model 
        that parameterizes the torque and thrust coefficients of the propeller using the advance ratio. 
        See ECE163_PropellerCheatSheet.pdf for details. Note: If the prop speed Omega is imaginary, then set it to 100.0.
        
        :param Va: The vehicle airspeed [m/s].
        :param Throttle: Throttle input [0-1].
        :return: Fx_prop [N], Mx_prop [N-m].
        """

        KT = 60 / (2 * math.pi * VPC.KV)
        KE = KT
        V_in = VPC.V_max * Throttle
        a = VPC.rho * VPC.D_prop ** 5 * VPC.C_Q0 / (4 * math.pi ** 2)
        b = VPC.rho * VPC.D_prop ** 4 * Va * VPC.C_Q1 / (2 * math.pi) + KT * KE / VPC.R_motor
        c = VPC.rho * VPC.D_prop ** 3 * Va ** 2 * VPC.C_Q2 - KT * V_in / VPC.R_motor + KT * VPC.i0
        if b ** 2 - 4 * a * c < 0:
            Omega = 100.0
        else:
            Omega = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        
        J = 2 * math.pi * Va / (Omega * VPC.D_prop)
        C_T = VPC.C_T0 + VPC.C_T1 * J + VPC.C_T2 * J ** 2
        C_Q = VPC.C_Q0 + VPC.C_Q1 * J + VPC.C_Q2 * J ** 2
        Fprop = VPC.rho * Omega ** 2 * VPC.D_prop ** 4 * C_T / (4 * math.pi ** 2)
        Mprop = - VPC.rho * Omega ** 2 * VPC.D_prop ** 5 * C_Q / (4 * math.pi ** 2)
        return Fprop, Mprop

    def Update(self, controls):
        """
        Function that uses the current state (internal), wind (internal), and controls (inputs) to calculate the forces, 
        and then do the integration of the full 6-DOF non-linear equations of motion. Wraps the VehicleDynamicsModel 
        class as well as the windState internally. The Wind and the vehicleState are maintained internally.
        
        :param controls: controlInputs class (Throttle, Elevator, Aileron, Rudder).
        """

        state = self.VDM.getVehicleState()
        totalForces = self.updateForces(state, controls, self.getWindModel().getWind())
        self.VDM.Update(totalForces)


    def aeroForces(self, state):
        """
        Function to calculate the Aerodynamic Forces and Moments using the linearized simplified force model 
        and the stability derivatives in VehiclePhysicalConstants.py file. Specifically does not include forces due 
        to control surface deflection. Requires airspeed (Va) in [m/s], angle of attack (alpha) in [rad] and sideslip 
        angle (beta) in [rad] from the state.
        
        :param state: Current vehicle state (need the velocities).
        :return: Aerodynamic forces, forcesMoments class.
        """

        C_L, C_D, C_M = self.CalculateCoeff_alpha(state.alpha)

        # Beard, Equation 4.6
        try:
            F_lift = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * (
                C_L + VPC.CLq * VPC.c / (2 * state.Va) * state.q
                )
        except:
            F_lift = 0.0

        # Beard, Equation 4.7
        try:
            F_drag = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * (
                C_D + VPC.CDq * VPC.c / (2 * state.Va) * state.q
            )
        except:
            F_drag = 0.0

        # Beard, Equation 4.5
        try:
            m = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * VPC.c * (
            VPC.CM0 + VPC.CMalpha * state.alpha + VPC.CMq * VPC.c / (2 * state.Va) * state.q
            )
        except:
            m = 0.0

        # Beard, Equation 4.14
        try:
            f_y = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * (
            VPC.CY0 + VPC.CYbeta * state.beta + VPC.CYp * VPC.b / (2 * state.Va) * state.p + VPC.CYr * VPC.b / (2 * state.Va) * state.r
            )
        except:
            f_y = 0.0
        
        # Beard, Equation 4.15
        try:
            l = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * VPC.b * (
            VPC.Cl0 + VPC.Clbeta * state.beta + VPC.Clp * VPC.b / (2 * state.Va) * state.p + VPC.Clr * VPC.b / (2 * state.Va) * state.r
            )
        except:
            l = 0.0

        # Beard, Equation 4.16
        try:
            n = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * VPC.b * (
            VPC.Cn0 + VPC.Cnbeta * state.beta + VPC.Cnp * VPC.b / (2 * state.Va) * state.p + VPC.Cnr * VPC.b / (2 * state.Va) * state.r
            )
        except:
            n = 0.0

        # Rotate forces into body frame
        R = [[math.cos(state.alpha), -math.sin(state.alpha)], 
             [math.sin(state.alpha), math.cos(state.alpha)]]
        F = MatrixMath.multiply(R, [[-1 * F_drag], [-1 * F_lift]])

        return Inputs.forcesMoments(Fx=F[0][0], Fy=f_y, Fz=F[1][0], Mx=l, My=m, Mz=n)

    def controlForces(self, state, controls):
        """
        Function to calculate aerodynamic forces from control surface deflections (including throttle) 
        using the linearized aerodynamics and simplified thrust model. Requires airspeed (Va) in [m/s] 
        and angle of attack (alpha) in [rad] both from state.Va and state.alpha respectively.
        
        :param state: Current vehicle state (need the velocities).
        :param controls: Inputs to aircraft - controlInputs().
        :return: Control surface forces, forcesMoments class.
        """

        C_L, C_D, C_M = self.CalculateCoeff_alpha(state.alpha)
        
        # Beard, Equation 4.5
        try:
            m = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * VPC.c * (
            VPC.CMdeltaE * controls.Elevator
            )
        except:
            m = 0.0

        # Beard, Equation 4.6
        try:
            F_lift = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * (
                VPC.CLdeltaE * controls.Elevator
                )
        except:
            F_lift = 0.0

        # Beard, Equation 4.7
        try:
            F_drag = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * (
                VPC.CDdeltaE * controls.Elevator
            )
        except:
            F_drag = 0.0

        # Beard, Equation 4.14
        try:
            f_y = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * (
            VPC.CYdeltaA * controls.Aileron + VPC.CYdeltaR * controls.Rudder
            )
        except:
            f_y = 0.0

        # Beard, Equation 4.15
        try:
            l = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * VPC.b * (
            VPC.CldeltaA * controls.Aileron + VPC.CldeltaR * controls.Rudder
            )
        except:
            l = 0.0

        # Beard, Equation 4.16
        try:
            n = 0.5 * VPC.rho * state.Va ** 2 * VPC.S * VPC.b * (
            VPC.CndeltaA * controls.Aileron + VPC.CndeltaR * controls.Rudder
            )
        except:
            n = 0.0
        
        Fprop, Mprop = self.CalculatePropForces(state.Va, controls.Throttle)

        # Rotate forces into body frame
        R = [[math.cos(state.alpha), -math.sin(state.alpha)],
                [math.sin(state.alpha), math.cos(state.alpha)]]
        F = MatrixMath.multiply(R, [[-1 * F_drag], [-1 * F_lift]])

        Fx, Mx = Fprop + F[0][0], Mprop + l
        return Inputs.forcesMoments(Fx=Fx, Fy=f_y, Fz=F[1][0], Mx=Mx, My=m, Mz=n)
        

    def getVehicleDynamicsModel(self):
        """
        Wrapper function to return the vehicle dynamics model handle.
        
        :return: vehicleDynamicsModel, from VehicleDynamicsModel class.
        """
        return self.VDM

    def getVehicleState(self):
        """
        Wrapper function to return vehicle state from module.
        
        :return: Vehicle state class.
        """
        return self.VDM.state
    
    def getWindModel(self):
        """
        Wrapper function to return the windModel.

        :return: windModel, from WindModel class.
        """
        return self.windModel

    def gravityForces(self, state):
        """
        Function to project gravity forces into the body frame. Uses the gravity constant g0 from physical constants 
        and the vehicle mass. Fg = m * R * [0 0 g0]'.
        
        :param state: Current vehicle state (need the rotation matrix).
        :return: Gravity forces, forcesMoments class.
        """
        Fg =  MatrixMath.scalarMultiply(VPC.mass, MatrixMath.multiply(state.R, [[0], [0], [VPC.g0]]))
        gravityForces = Inputs.forcesMoments(Fx=Fg[0][0], Fy=Fg[1][0], Fz=Fg[2][0])
        return gravityForces

    def reset(self):
        """
        Resets module to its original state so it can run again.
        
        :return: None.
        """
        self.VDM = VehicleDynamicsModel.VehicleDynamicsModel()
        self.VDM.state = States.vehicleState(pd=self.initialHeight, u=self.initialSpeed)
        self.windModel = WindModel.WindModel()

    def setVehicleState(self, state):
        """
        Wrapper function to set the vehicle state from outside module.
        
        :param state: Class of vehicleState.
        :return: None.
        """
        self.VDM.state = state

    def setWindModel(self, windModel):
        """
        Wrapped function to set the windModel
        
        :param windModel: from windModel class
        :return: None
        """
        self.windModel = windModel

    def updateForces(self, state, controls, wind=None):
        """
        Function to update all of the aerodynamic, propulsive, and gravity forces and moments. All calculations
        required to update the forces are included. state is updated with new values for airspeed, angle of attack,
        and sideslip angles (see class definition for members)

        :param state: current vehicle state
        :param controls: current vehicle control surface deflections
        :param wind: current environmental wind. If not specified, defaults to 0 windspeed
        :return: total forces, forcesMoments class
        """
        if wind is None:
            if state.u == 0 and state.v == 0 and state.w == 0:
                state.Va = 0
                state.alpha = 0
                state.beta = 0
            else:
                state.Va = math.sqrt(state.u ** 2 + state.v ** 2 + state.w ** 2)
                state.alpha = math.atan2(state.w, state.u)
                state.beta = math.asin(state.v / state.Va)
        else:
            state.Va, state.alpha, state.beta = self.CalculateAirspeed(state, wind)

        gravityForces = self.gravityForces(state)
        aeroForces = self.aeroForces(state)
        controlForces = self.controlForces(state, controls)

        Fx = gravityForces.Fx + aeroForces.Fx + controlForces.Fx
        Fy = gravityForces.Fy + aeroForces.Fy + controlForces.Fy
        Fz = gravityForces.Fz + aeroForces.Fz + controlForces.Fz
        Mx = gravityForces.Mx + aeroForces.Mx + controlForces.Mx
        My = gravityForces.My + aeroForces.My + controlForces.My
        Mz = gravityForces.Mz + aeroForces.Mz + controlForces.Mz

        return Inputs.forcesMoments(Fx=Fx, Fy=Fy, Fz=Fz, Mx=Mx, My=My, Mz=Mz)