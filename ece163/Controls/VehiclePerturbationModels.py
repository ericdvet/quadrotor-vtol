"""
Author: Eric Vetha (evetha@ucsc.edu)

File that contains all the functions required to create the linearized perturbation models from the stability derivatives
and the non-linear model (using numerical gradients). The linearization (both in terms of state space and transfer
function) is done about the trim state that is input to the functions. All models are using perturbations about nominal
trim states and inputs (e.g.: deltaElevator is change about trim elevator angle).
"""

import math
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath

def CreateTransferFunction(trimState, trimInputs):
    """
    Function to fill the transfer function parameters used for the successive loop closure from the given trim state and
    trim inputs. Note that these parameters will be later used to generate actual control loops. Vehicle Perturbation
    models are developed using the trim state and inputs. Models for transfer function parameters and state space
    implementations are calculated using constants in VehiclePhysicalParameters and the input trim state and trim
    control inputs. Results are returned as a Linearized.transferFunction class.
    
    :param trimState: vehicle trim state, as calculated in VehicleTrim code
    :param trimInputs: vehicle trim inputs, as calculated in VehicleTrim code
    :return:  transferFunction, from Linearized.transferFunctions class
    """

    Va_trim = trimState.Va
    alpha_trim = trimState.alpha
    beta_trim = trimState.beta
    gamma_trim = trimState.pitch - trimState.alpha
    theta_trim = trimState.pitch
    phi_trim = trimState.roll   

    a_phi1 = -(1/2) * VPC.rho * Va_trim**2 * VPC.S * VPC.b * VPC.Cpp * VPC.b / (2 * Va_trim)
    a_phi2 = (1/2) * VPC.rho * Va_trim**2 * VPC.S * VPC.b * VPC.CpdeltaA

    a_beta1 = - VPC.rho * Va_trim * VPC.S / (2 * VPC.mass) * VPC.CYbeta
    a_beta2 = VPC.rho * Va_trim * VPC.S / (2 * VPC.mass) * VPC.CYdeltaR

    a_theta1 = -VPC.rho * trimState.Va**2 * VPC.c * VPC.S * VPC.CMq * VPC.c / (4 * VPC.Jyy * trimState.Va)
    a_theta2 = -VPC.rho * trimState.Va**2 * VPC.c * VPC.S * VPC.CMalpha / (2 * VPC.Jyy)
    a_theta3 = VPC.rho * trimState.Va**2 * VPC.c * VPC.S * VPC.CMdeltaE / (2 * VPC.Jyy)

    Va = math.hypot(trimState.u, trimState.v, trimState.w)
    a_V1 = -(VPC.rho * Va * VPC.S * 
             (-VPC.CD0 - VPC.CDalpha * trimState.alpha - VPC.CDdeltaE * trimInputs.Elevator) + 
             dThrust_dVa(Va, trimInputs.Throttle)) / VPC.mass
    a_V2 = dThrust_dThrottle(Va, trimInputs.Throttle, epsilon=0.01) / VPC.mass
    a_V3 = VPC.g0 * math.cos(trimState.pitch - trimState.alpha)

    return Linearized.transferFunctions(Va_trim, alpha_trim, beta_trim, gamma_trim, theta_trim, phi_trim, 
                                        a_phi1, a_phi2, a_beta1, a_beta2, a_theta1, a_theta2, a_theta3, a_V1, a_V2, a_V3)


def dThrust_dThrottle(Va, Throttle, epsilon=0.01):
    """
    def dThrust_dThrottle(Va, Throttle, epsilon=0.01): Function to calculate the numerical partial derivative of pro
    peller thrust to change in throttle setting using the actual prop function from complex propeller model (inside the
    VehicleAerodynamicsModel class)
    
    :param Va: vehicle trim airspeed [m/s]
    :param Throttle: trim throttle setting [0-1]
    :param epsilon: step to take for perturbation in throttle setting
    :return: dTdDeltaT: partial derivative [N/PWM]
    """

    VAM = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    dTdDeltaT = (VAM.CalculatePropForces(Va, Throttle + epsilon)[0]
                 - VAM.CalculatePropForces(Va, Throttle)[0]) / (epsilon)
    return dTdDeltaT

def dThrust_dVa(Va, Throttle, epsilon=0.5):
    """
    def dThrust_dVa(Va, Throttle, epsilon=0.5): Function to calculate the numerical partial derivative of propeller
    thrust to changeinairspeedusingtheactualpropfunctionfromcomplexpropellermodel(insidetheVehicleAero
    dynamicsModel class)
    
    :param Va: vehicle trim airspeed [m/s]
    :param Throttle: trim throttle setting [0-1]
    :param epsilon: step to take for perturbation in Velocity
    :return: dTdVa: partial derivative [N-s/m]
    """

    VAM = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    dTdVa = (VAM.CalculatePropForces(Va + epsilon, Throttle)[0] 
             - VAM.CalculatePropForces(Va, Throttle)[0]) / (epsilon)
    return dTdVa