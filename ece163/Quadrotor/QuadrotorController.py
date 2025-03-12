"""
Description: This module contains the QuadrotorController class, which is used to control the quadrotor's position and orientation. We use a PID controller based on the work in "Modelling and PID Controller Design for a Quadrotor Unmanned Air Vehicle" (Salih et al. 20??) 
"""

# delete these notes when done

import QuadrotorPhysicalConstants
import numpy as np
from simple_pid import PID
import control as ctrl

def QuadrotorInputs(self, thruster_commands, quad):        
    Th_1 = thruster_commands[0]     # The thrusts generated by four rotors
    Th_2 = thruster_commands[1] 
    Th_3 = thruster_commands[2]
    Th_4 = thruster_commands[3]

    l = quad['d'] / 2                                           # Half length of the helicopter
    m = quad['M']                                               # Total mass of the helicopter
    I_1, I_2, I_3 = quad['J'][0], quad['J'][1], quad['J'][2]    # Moments of inertia with respect to the axes
    K_4, K_5, K_6 = quad['K'][3], quad['K'][4], quad['K'][5]    # The drag coefficients
    C = 1                                                       # Force to moment scaling factor

    u1 = (Th_1 + Th_2 + Th_3 + Th_4) / m           # Vertical thrust generated by the four rotors
    u2 = l * (Th_1 - Th_2 + Th_3 - Th_4) / I_1     # Pitching moment
    u3 = l * (Th_1 + Th_2 - Th_3 - Th_4) / I_2     # Yawing moment
    u4 = C * (Th_1 - Th_2 + Th_3 - Th_4) / I_3     # Rolling moment

    return u1, u2, u3, u4

def QuadrotorEquationsOfMotion(x, quad, u):

    K_1, K_2, K_3 = quad['K'][0], quad['K'][1], quad['K'][2]    # The drag coefficients
    K_4, K_5, K_6 = quad['K'][3], quad['K'][4], quad['K'][5]    # The drag coefficients
    m = quad['M']                                               # Total mass of the helicopter
    g = quad['g']                                               # Gravity
    l = quad['d'] / 2                                           # Half length of the helicopter
    I_1, I_2, I_3 = quad['J'][0], quad['J'][1], quad['J'][2]    # Moments of inertia with respect to the axes

    # Unpacking the state vector
    x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r = x
    theta_dot = q
    phi_dot = p
    psi_dot = r

    # Unpacking the control inputs
    u_1 = u[0] 
    u_2 = u[1]  
    u_3 = u[2]  
    u_4 = u[3]  

    # Equation 2 (Salih et al. 2010)
    x_ddot = u_1 * (np.sin(phi) * np.sin(theta) * np.cos(psi) +         # Forward position in earth axes
                    np.sin(phi) * np.sin(psi)) - K_1 * x_dot / m 
    y_ddot = u_1 * (np.sin(phi) * np.sin(psi) * np.cos(psi) -           # Lateral position in earth axes
                    np.cos(phi) * np.sin(psi)) - K_2 * y_dot / m
    z_ddot = u_1 * (np.cos(phi) * np.cos(psi)) - g - K_3 * z_dot / m    # Vertical position in earth axes

    # Equation 4 (lih et al. 2010)
    theta_ddot = u_2 - l * K_4 * theta_dot / I_1
    phi_ddot = u_4 - l * K_5 * phi_dot / I_2
    psi_ddot = u_3 - l * K_6 * psi_dot / I_3

    return x_ddot, y_ddot, z_ddot, phi_ddot, theta_ddot, psi_ddot

# TODO: Everything can be a class method there's a lot of repeated code

def AngleOfAttack(x, x_target):

    x_d, y_d, z_d = x_target[0], x_target[1], x_target[2]
    x, y, z = x[0], x[1], x[2]

    theta_d = np.arctan2(y_d - y, x_d - x)                              # Desired roll angle
    phi_d = np.arctan2(z_d - z, np.sqrt((x_d - x)**2 + (y_d - y)**2))   # Desired yaw angle

# Redoing the above because I don't think it's correct
# Let's start from scratch
# We have reference trajectory and the environment

class Trajectory:

    def __init__(self, x, x_target, t):
        
        # We need to constantly output the trajectory's yaw, position, and velocity
        # which means that we need to constantly update the trajectory because
        # the derivative is required for the controller

        self.yaw = np.arctan2(x_target[1] - x[1], x_target[0] - x[0])
        self.position = x[:3]
        self.velocity = np.zeros(3)
        self.t = t
    
    def update(self,  x, x_target, dt):

        self.yaw = np.arctan2(x_target[1] - x[1], x_target[0] - x[0])
        self.t += dt
        self.velocity = (x[:3] - self.position) / dt
        self.position = x[:3]

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
# TODO: Does this need to be a class?

def PositionControl(m: Environment, traj: Trajectory, t):

    # Body Frame
    R_BI = calculateRBI(m.roll, m.pitch, m.yaw)

    # Compute position error
    pos_error = traj.position - np.array([m.px, m.py, m.pz])

    # PID 
    kp_xy2ypr = 8           # Unhard code this
    ki_xy2ypr = 0.04
    kd_xy2ypr = 3.2
    filtD_xy2ypr = 100

    XY2YPR = PID(Kp = kp_xy2ypr, Ki = ki_xy2ypr, Kd = kd_xy2ypr)
    px = XY2YPR(pos_error[0])
    py = XY2YPR(pos_error[1])
    pz = XY2YPR(pos_error[2])

    i = R_BI @ np.array([px, py, pz])

    filter_cnst = 0.005  # TODO: unhard code this
    num = [1]
    den = [filter_cnst, 1]
    filter = ctrl.TransferFunction(num, den)

    o = np.zeros(3)
    t, o[0], _ = ctrl.forced_response(filter, U=i[0])
    t, o[1], _ = ctrl.forced_response(filter, U=i[1])
    t, o[2], _ = ctrl.forced_response(filter, U=i[2])

    # Body Frame

    err2rp = 2.4

    o[0] = 1/9.81 * err2rp * o[0]
    if o[0] > np.pi/3:
        o[0] = np.pi/3
    elif o[0] < -np.pi/3:
        o[0] = -np.pi/3
    
    o[1] = 1/9.81 * err2rp * o[1]
    if o[1] > np.pi/3:
        o[1] = np.pi/3
    elif o[1] < -np.pi/3:
        o[1] = -np.pi/3

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
