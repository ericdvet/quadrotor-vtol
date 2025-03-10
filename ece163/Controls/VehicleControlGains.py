"""
Author: Eric Vetha (evetha@ucsc.edu)

File that contains the machinery to calculate the gains for successive loop closure for both lateral and longitudinal modes
of the UAV. The design choices for each look are going to be the bandwidth of the loop, along with the damping, as
this will determine the gains for each part of the successive loop. See Beard Chapter 6. Note that the gain calculations
are bi-directional, in that you can also tweak the gains and pull out the natural frequencies and damping as well.

Linear system model parameters are calculated around a given trim condition uniquely determined by a set of desired
airspeed, flight path angle, and turn radius (e.g.: Va [m/s], gamma [rad], R [m]). These are calculated using the
VehiclePerturbationsModels file, and the trim conditions are determined using the VehicleTrim file. Note that the
linearized equations assume that the control inputs are perturbations about the trim values.
"""

import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations

def computeGains(tuningParameters=Controls.controlTuning(), linearizedModel=Linearized.transferFunctions()):
    """
    Function to compute the control gains using the tuning parameters outlined in Beard Chapter 6. Both the lateral
    and longitudinal gains are calculated. No check is made for frequency separation. Transfer function input comes
    form the VehiclePerturbationModels which rely on the VehicleTrim.py (provided) to compute the trim values.
    Note: no effort has been made to trap arithmetic errors. You code could be more robust by wrapping the entire
    method in atry/except block and raising the exception when it failed. Given that your previous code should work,
    this is not required.

    :param tuningParameters:  class controlTuning from Containers.Controls
    :param linearizedModel: class transferFunction from Containers.Linearized
    :return: class controlGains from Containers.Controls
    """

    controlGains = Controls.controlGains()

    controlGains.kp_roll = tuningParameters.Wn_roll ** 2 / linearizedModel.a_phi2
    controlGains.kd_roll = (2 * tuningParameters.Zeta_roll * tuningParameters.Wn_roll - linearizedModel.a_phi1) / linearizedModel.a_phi2
    controlGains.ki_roll = 1e-3 # ???

    controlGains.kp_sideslip = (2 * tuningParameters.Zeta_sideslip * tuningParameters.Wn_sideslip - linearizedModel.a_beta1) / linearizedModel.a_beta2
    controlGains.ki_sideslip = tuningParameters.Wn_sideslip** 2 / linearizedModel.a_beta2
    
    controlGains.kp_course = 2 * tuningParameters.Zeta_course * tuningParameters.Wn_course * linearizedModel.Va_trim / VPC.g0
    controlGains.ki_course = tuningParameters.Wn_course ** 2 * linearizedModel.Va_trim / VPC.g0 

    controlGains.kp_pitch = (tuningParameters.Wn_pitch ** 2 - linearizedModel.a_theta2) / linearizedModel.a_theta3
    controlGains.kd_pitch = (2 * tuningParameters.Zeta_pitch * tuningParameters.Wn_pitch - linearizedModel.a_theta1) / linearizedModel.a_theta3

    K_theta_DC = controlGains.kp_pitch * linearizedModel.a_theta3 / (linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)
    controlGains.kp_altitude = 2 * tuningParameters.Zeta_altitude * tuningParameters.Wn_altitude / (K_theta_DC * linearizedModel.Va_trim)
    controlGains.ki_altitude = tuningParameters.Wn_altitude ** 2 / (K_theta_DC * linearizedModel.Va_trim)

    controlGains.kp_SpeedfromThrottle = (2 * tuningParameters.Zeta_SpeedfromThrottle * tuningParameters.Wn_SpeedfromThrottle - linearizedModel.a_V1) / linearizedModel.a_V2
    controlGains.ki_SpeedfromThrottle = tuningParameters.Wn_SpeedfromThrottle ** 2 / linearizedModel.a_V2

    controlGains.kp_SpeedfromElevator = (linearizedModel.a_V1 - 2 * tuningParameters.Zeta_SpeedfromElevator * tuningParameters.Wn_SpeedfromElevator) / (K_theta_DC * VPC.g0)
    controlGains.ki_SpeedfromElevator = - tuningParameters.Wn_SpeedfromElevator ** 2 / (K_theta_DC * VPC.g0)

    return controlGains # JFC...

def computeTuningParameters(controlGains=Controls.controlGains(), linearizedModel=Linearized.transferFunctions()):
    """
    Function to compute the tuning parameters given the gains in the
    successive loop closure, needs a try block to deal with taking square root of negative number. Function should
    never fail, if an exception occurs, return an empty (inited) turningParameters class. Transfer function input
    comes form the VehiclePerturbationModels which rely on the VehicleTrim.py (provided) to compute the trim
    values.

    :param controlGains: class controlGains from Containers.Controls
    :param linearizedModel: class transferFunction from Containers.Linearized
    :return: class controlTuning from Containers.Controls
    """

    tuningParameters = Controls.controlTuning()

    try:
        tuningParameters.Wn_roll = math.sqrt(controlGains.kp_roll * linearizedModel.a_phi2)
        tuningParameters.Zeta_roll = (linearizedModel.a_phi1 + controlGains.kd_roll * linearizedModel.a_phi2) / (2 * tuningParameters.Wn_roll)
    except:
        tuningParameters.Wn_roll = 0
        tuningParameters.Zeta_roll = 0
    
    try:
        tuningParameters.Wn_course = math.sqrt(controlGains.ki_course * VPC.g0 / linearizedModel.Va_trim)
        tuningParameters.Zeta_course = (controlGains.kp_course * VPC.g0)/ (2 * linearizedModel.Va_trim * tuningParameters.Wn_course)
    except:
        tuningParameters.Wn_course = 0
        tuningParameters.Zeta_course = 0

    try:
        tuningParameters.Wn_sideslip = math.sqrt(controlGains.ki_sideslip * linearizedModel.a_beta2)
        tuningParameters.Zeta_sideslip = (controlGains.kp_sideslip * linearizedModel.a_beta2 + linearizedModel.a_beta1) / (2 * tuningParameters.Wn_sideslip)
    except:
        tuningParameters.Wn_sideslip = 0
        tuningParameters.Zeta_sideslip = 0

    try:
        tuningParameters.Wn_pitch = math.sqrt(linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)
        tuningParameters.Zeta_pitch = (linearizedModel.a_theta1 + controlGains.kd_pitch * linearizedModel.a_theta3) / (2 * tuningParameters.Wn_pitch)
    except:
        tuningParameters.Wn_pitch = 0
        tuningParameters.Zeta_pitch = 0
    
    K_theta_DC = controlGains.kp_pitch * linearizedModel.a_theta3 / (linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)

    try:
        tuningParameters.Wn_altitude = math.sqrt(controlGains.ki_altitude * K_theta_DC * linearizedModel.Va_trim)
        tuningParameters.Zeta_altitude = (controlGains.kp_altitude * K_theta_DC * linearizedModel.Va_trim) / (2 * tuningParameters.Wn_altitude)
    except:
        tuningParameters.Wn_altitude = 0
        tuningParameters.Zeta_altitude = 0
    
    try:
        tuningParameters.Wn_SpeedfromThrottle = math.sqrt(controlGains.ki_SpeedfromThrottle * linearizedModel.a_V2)
        tuningParameters.Zeta_SpeedfromThrottle = (controlGains.kp_SpeedfromThrottle * linearizedModel.a_V2 + linearizedModel.a_V1) / (2 * tuningParameters.Wn_SpeedfromThrottle)
    except:
        tuningParameters.Wn_SpeedfromThrottle = 0
        tuningParameters.Zeta_SpeedfromThrottle = 0

    try:
        tuningParameters.Wn_SpeedfromElevator = math.sqrt(-controlGains.ki_SpeedfromElevator * K_theta_DC * VPC.g0)
        tuningParameters.Zeta_SpeedfromElevator = (controlGains.kp_SpeedfromElevator * K_theta_DC * VPC.g0 - linearizedModel.a_V1) / (- 2 * tuningParameters.Wn_SpeedfromElevator)
    except:
        tuningParameters.Wn_SpeedfromElevator = 0
        tuningParameters.Zeta_SpeedfromElevator = 0

    return tuningParameters  