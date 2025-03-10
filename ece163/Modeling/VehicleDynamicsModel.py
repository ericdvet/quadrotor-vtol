"""
Author: Eric Vetha (evetha@ucsc.edu)

This module is whereall of the vehicle dynamics are computed for the simulation. It includes the kinematics of both the translational and rotational dynamics. Included are both the derivative, and the integration functions, and the rotations of forces to the body frame
"""

import math
from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC
import numpy as np
import scipy.linalg as la

class VehicleDynamicsModel:
    """
    :param: state - the current state of the vehicle, as an instance of States.vehicleState
    :param: dot - the current time-derivative of the state, as an instance of States.vehicleState
    :param: dT - the timestep that this object uses when Update()ing.
    """
    
    def __init__(self, dT = 0.01):
        """
        Initializes the class, and sets the time step (needed for Rexp and integration). Instantiates attributes for vehicle state, and time derivative of vehicle state.
        
        :param: dT - defaults to VPC.dT
        :return: None
        """
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
        self.dT = dT
    
    def reset(self):
        """
        Reset the Vehicle state to initial conditions
        
        :return: None
        """
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
        
    def getVehicleState(self):
        """
        Getter method to read the vehicle state
        
        :return: state (from class vehicleState)
        """
        return self.state
    
    def setVehicleState(self, state):
        """
        Setter method to write the vehicle state
        
        :param: state - state to be set ( an instance of Containers.States.vehicleState)
        :return: None
        """
        self.state = state
    
    def getVehicleDerivative(self):
        """
        Getter method to read the vehicle state time derivative
        
        :return: dot ( an instance of Containers.States.vehicleState)
        """
        return self.dot
    
    def setVehicleDerivative(self, dot):
        """
        Setter method to write the vehicle state 
        
        :param: state - state to be set ( an instance of Containers.States.vehicleState)
        :return: None
        """
        self.dot = dot
    
    def Update(self, forcesMoments):
        """
        Function that implements the integration such that the state is updated using the forces and moments passed in as the arguments (dT is internal from the member). State is updated in place (self.state is updated). Use getVehicleState to retrieve state. Time step is defined in VehiclePhyscialConstants.py
        
        :param: forcesMoments - forces [N] and moments [N-m] defined in forcesMoments class
        :return: None
        """
        dot = self.derivative(self.state, forcesMoments)
        self.state = self.IntegrateState(self.dT, self.state, dot)
        
    def derivative(self, state, forcesMoments):
        """
        Function to compute the time-derivative of the state given body frame forces and moments
        
        :param: state - state to differentiate, as a States.vehicleState object
        :param: forcesMoments - forces [N] and moments [N-m] as an Inputs.forcesMoments object
        :return: the current time derivative, in the form of a States.vehicleState object
        """
        
        dot = States.vehicleState()
        
        # p_n, p_e, p_d
        u, v, w = state.u, state.v, state.w
        yaw, pitch, roll = state.yaw, state.pitch, state.roll
        R_v2b = euler2dcm(yaw, pitch, roll)
        R_b2v = np.transpose(R_v2b)
        d_p = R_b2v @ np.array([[u], [v], [w]])
        d_pn, d_pe, d_pd = d_p[0][0], d_p[1][0], d_p[2][0]
        dot.pn, dot.pe, dot.pd = d_pn, d_pe, d_pd
        
        # yaw, pitch, roll 
        p, q, r = state.p, state.q, state.r
        d_euler = np.array([
            [1, np.sin(roll)*np.tan(pitch), np.cos(roll)*np.tan(pitch)],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll)/np.cos(pitch), np.cos(roll)/np.cos(pitch)]
        ]) @ np.array([[p], [q], [r]])
        d_yaw, d_pitch, d_roll = d_euler[2][0], d_euler[1][0], d_euler[0][0]
        dot.yaw, dot.pitch, dot.roll = d_yaw, d_pitch, d_roll
        
        # u, v, w
        m = VPC.mass
        fx, fy, fz = forcesMoments.Fx, forcesMoments.Fy, forcesMoments.Fz
        if fx + fy + fz != 0:
            d_vel = np.array([
                [r*v - q*w],
                [p*w - r*u],
                [q*u - p*v]
            ]) + 1 / m * np.array([[fx], [fy], [fz]])
        else:
            d_vel = np.array([
                [r*v - q*w],
                [p*w - r*u],
                [q*u - p*v]
            ])
        d_u, d_v, d_w = d_vel[0][0], d_vel[1][0], d_vel[2][0]
        dot.u, dot.v, dot.w = d_u, d_v, d_w
        
        # p, q, r
        J = VPC.Jbody
        m = np.array([forcesMoments.Mx, forcesMoments.My, forcesMoments.Mz])
        omega = np.array([p, q, r])
        pqr_dot = VPC.JinvBody @ (m - np.cross(omega, J @ omega))
        p_dot, q_dot, r_dot = pqr_dot[0], pqr_dot[1], pqr_dot[2]
        dot.p, dot.q, dot.r = p_dot, q_dot, r_dot
        
        R_dot = -1 * skew(p, q, r) @ np.array(state.R)
        R_dot = R_dot.tolist()
        dot.R = R_dot
        
        return dot
    
    def ForwardEuler(self, dT, state, dot):
        """
        Function to do the simple forwards integration of the state using the input derivative (“dot”- generated elsewhere by the derivative function). State is integrated using the update formula X_{k+1} = X_{k} + dX/dt * dT. The updated state is returned by the function. The state is held internally as an attribute of the class.
        
        :param: dT - the timestep over which to forward integrate
        :param: state - the initial state to integrate, as an instance of State.vehicleState
        :param: dot - the time-derivative of the state for performing integration, as an instance of State.vehicleState
        :return: new state, advanced by a timestep of dT (defined in States.vehicleState class)
        """
        
        state_next_timestep = States.vehicleState()
        state_next_timestep.yaw = state.yaw + dot.yaw * dT
        state_next_timestep.pitch = state.pitch + dot.pitch * dT
        state_next_timestep.roll = state.roll + dot.roll * dT
        state_next_timestep.u = state.u + dot.u * dT
        state_next_timestep.v = state.v + dot.v * dT
        state_next_timestep.w = state.w + dot.w * dT
        state_next_timestep.pn = state.pn + dot.pn * dT
        state_next_timestep.pe = state.pe + dot.pe * dT
        state_next_timestep.pd = state.pd + dot.pd * dT
        state_next_timestep.p = state.p + dot.p * dT
        state_next_timestep.q = state.q + dot.q * dT
        state_next_timestep.r = state.r + dot.r * dT
        return state_next_timestep
        
    def Rexp(self, dT, state, dot):
        """
        Calculates the matrix exponential exp(-dT*[omega x]), which can be used in the closed form solution for the DCMintegration from body-fixed rates. See the document (ECE163_AttitudeCheatSheet.pdf) for details.
        
        :param: dT - time step [sec]
        :param: state - the vehicle state, in the form of a States.vehicleState object
        :param: dot - the state derivative, in the form of a States.vehicleState object
        :return: Rexp: the matrix exponential to update the state
        """
        
        p, q, r = state.p + 0.5 * dT * dot.p, state.q + 0.5 * dT * dot.q, state.r + 0.5 * dT * dot.r
        
        magnitude = la.norm([p, q, r])
        if magnitude > 0.1: # 0.1 passes 48 tests
            sin_part = math.sin(dT * magnitude) / magnitude
            cos_part = (1 - math.cos(dT * magnitude)) / magnitude**2
        else:
            sin_part = dT - (dT**3 * magnitude**2) / 6 + (dT**5 * magnitude**4) / 120
            cos_part = dT**2 / 2 - (dT**4 * magnitude**2) / 24 + (dT**6 * magnitude**4) / 720
        
        Rexp = np.eye(3) - sin_part * skew(p, q, r) + cos_part * skew(p, q, r) @ skew(p, q, r)
            
        return Rexp.tolist()
        
    def IntegrateState(self, dT, state, dot):
        """
        Updates the state given the derivative, and a time step. Attitude propagation is implemented as a DCM matrix exponential solution, all other state params are advanced via forward euler integration [x]k+1 = [x]k + xdot*dT. The integrated state is returned from the function. All derived variables in the state (e.g.: Va, alpha, beta, chi) should be copied from the input state to the returned state.
         
        chi for the newstate is atan2( pe_dot, pn_dot). This will be needed later in the course.

        :param: Time step [s]
        :param: state - the initial state to integrate, as an instance of State.vehicleState
        :param: dot - the time-derivative of the state for performing integration, as an instance of State.vehicleState
        :return:  new state, advanced by a timestep of dT, returned as an instance of the States.vehicleState class
        """
        
        newState = self.ForwardEuler(dT, state, dot)
        newState.R = np.array(self.Rexp(dT, state, dot)) @ np.array(state.R)
        newState.R = newState.R.tolist()
        euler = Rotations.dcm2Euler(newState.R)
        newState.yaw, newState.pitch, newState.roll = euler[0], euler[1], euler[2]
        
        newState.alpha = state.alpha
        newState.beta = state.beta
        newState.Va = state.Va
        newState.chi = math.atan2(dot.pe, dot.pn)
        
        return newState

# =============================================================================
# Helper functions
# =============================================================================
def euler2dcm(yaw, pitch, roll):
    """
    Calculate the transpose of the Direction Cosine Matrix (DCM) from Euler angles.
    
    :param yaw: Yaw angle in radians
    :param pitch: Pitch angle in radians
    :param roll: Roll angle in radians
    :return: 3x3 numpy array
    """
    dcm = np.array([
        [np.cos(yaw)*np.cos(pitch), np.sin(yaw)*np.cos(pitch), -np.sin(pitch)],
        [np.cos(yaw)*np.sin(pitch)*np.sin(roll) - np.sin(yaw)*np.cos(roll), np.sin(yaw)*np.sin(pitch)*np.sin(roll) + np.cos(yaw)*np.cos(roll), np.cos(pitch)*np.sin(roll)],
        [np.cos(yaw)*np.sin(pitch)*np.cos(roll) + np.sin(yaw)*np.sin(roll), np.sin(yaw)*np.sin(pitch)*np.cos(roll) - np.cos(yaw)*np.sin(roll), np.cos(pitch)*np.cos(roll)]
    ])
    
    return dcm

def dcm2euler(dcm):
    """
    Calculate the Euler angles from the Direction Cosine Matrix (DCM).
    
    :param dcm: 3x3 numpy array
    :return: yaw, pitch, roll in radians
    """
    pitch = -np.arcsin(dcm[2, 0])
    roll = np.arctan2(dcm[2, 1], dcm[2, 2])
    yaw = np.arctan2(dcm[1, 0], dcm[0, 0])
    
    return yaw, pitch, roll

def skew(p, q, r):
    """
    Calculate the skew symmetric matrix of angular velocities.
    
    :param p: Angular velocity about the x-axis
    :param q: Angular velocity about the y-axis
    :param r: Angular velocity about the z-axis
    :return: 3x3 numpy array
    """
    return np.array([
        [0, -r, q],
        [r, 0, -p],
        [-q, p, 0]
    ])