"""
Author: Eric Vetha (evetha@ucsc.edu)

Description: This module contains the QuadrotorModel class, which is a simulation of idealized X-4 Flyer II flight dynamics based upon Pounds et al. (2010). Much of the logic is inhereted from the Rolling Spider software package version 6.0.0. 
"""

import math
import numpy as np
import QuadrotorPhysicalConstants 
from Containers import States

class QuadrotorModel:
    def __init__(self, quad, dT = 0.01):
        """
        Initialization of the internal classes which are used to track the vehicle aerodynamics and dynamics.

        Args:
            quad (dict): A dictionary of quadrotor physical constants. This is defined in the QuadrotorPhysicalConstants module.
            dT (float): The time step for the simulation

        Returns:
            None
        """

        self.quad = quad
        self.dT = dT

        self.sys = np.zeros(12)
        x0 = np.array([0, 0, -0.046,   # z (position)
                            0, 0, 0,        # n (angular position)
                            0, 0, 0,        # v (velocity)
                            0, 0, 0])       # o (angular velocity)

        self.x = x0

    def update(self, t, x, u):
        """
        Function that uses the current internal state (x) and motor commands (u) to update the state of the quadrotor. 
        
        Args:
            x: The current state of the quadrotor. 
            u: The motor commands for the quadrotor. This is a 4 element array of motor speeds in RPM (I think).

        Returns:
            x: The updated state of the quadrotor.
        """

        self.sys, self.x0 = self.quadrotor_dynamics(x, u, 1, self.quad)
        x = self.x + self.sys * self.dT
        self.sys, self.x = self.quadrotor_dynamics(x, u, 2, self.quad)
        self.x = x
        return x


    def quadrotor_dynamics(self, x, u, flag, quad):
        """
        This function is a wrapper for the quadrotor dynamics model. It inherits a S-function style interface from the Rolling Spider software package (which, admittedly, doesn't make a lot of sense in Python).

        Args:
            x: The current state of the quadrotor. This is described in the quadrotor_dynamics function.
            u: The motor commands for the quadrotor. This is a 4 element array of motor speeds in RPM (I think).
            flag: A flag that tells the function what to do. 0 is initialization, 1 is derivative calculation, 2 is output calculation.
            quad: The physical constants of the quadrotor.
        """


        if flag == 1:                                         # Derivative Calculation
            sys = modelDerivatives(x, u, quad)
            x = []
        elif flag == 2:                                         # Output Calculation
            sys = modelOutputs(x)
            x = []
        else:                                                   # Error
            raise NotImplementedError
        
        return sys, x
    
    def wrapper_vehicleState(self):
        """
        Converts the quadrotor state vector to a vehicleState object for use in the simulation.

        Returns:
            vehicleState: A vehicleState object that represents the current state of the quadrotor.
        """

        vehicleState = States.vehicleState(pn = self.x[0], pe = self.x[1], pd = self.x[2], 
                                           u = self.x[6], v = self.x[7], w = self.x[8], 
                                           yaw = self.x[3], pitch = self.x[4], roll = self.x[5], 
                                           p = self.x[9], q = self.x[10], r = self.x[11])
        return vehicleState

def modelDerivatives(x, u, quad):
    """
    Calculate the state derivatives for the next timestep. Inherited from the Rolling Spider software package. If it ain't broke, don't fix it.

    Args:
        x: The current state of the quadrotor. 
        u: The motor commands for the quadrotor. This is a 4 element array of motor speeds in RPM (I think).
        quad: The physical constants of the quadrotor.
    
    Returns:
        sys: The state derivatives for the next timestep.
    """
    
    # CONSTANTS
    # Cardinal Direction Indices
    N = 0   # North
    E = 1   # East
    S = 2   # South
    W = 3   # West

    d = math.sqrt(2) / 2 * quad['d']

    D = np.array([                  # Rotor hub displacements
        [d, -d, quad['h']],
        [d, d, quad['h']],
        [-d, d, quad['h']],
        [-d, -d, quad['h']]
    ]).T

    e1 = np.array([1, 0, 0])        # Body-fixed frame references
    e2 = np.array([0, 1, 0])
    e3 = np.array([0, 0, 1])

    # EXTRACT STATES FROM U
    w = u

    # EXTRACT STATES FROM X
    z = x[0:3]      # position in {W}
    n = x[3:6]      # RPY angles {W}
    v = x[6:9]      # velocity in {W}
    o = x[9:12]     # angular velocity in {B} (wx, wy, wz) (FR)

    # PREPROCESS ROTATION AND WRONSKIAN MATRICES
    phi = n[0]      # yaw
    the = n[1]      # pitch
    psi = n[2]      # roll

    if abs(the) > math.pi / 2:
        raise ValueError('|pitch| greater than pi/2!')
    if abs(psi) > math.pi / 2:
        raise ValueError('|roll| greater than pi/2!')
    if z[2] > 0:
        raise ValueError('z greater than 0 (below ground)!')

    # R_phi @ R_the @ R_psi
    R_Body2World = np.array([
        [math.cos(the) * math.cos(phi),     math.sin(psi) * math.sin(the) * math.cos(phi) - math.cos(psi) * math.sin(phi),  math.cos(psi) * math.sin(the) * math.cos(phi) + math.sin(psi) * math.sin(phi)],
        [math.cos(the) * math.sin(phi),     math.sin(psi) * math.sin(the) * math.sin(phi) + math.cos(psi) * math.cos(phi),  math.cos(psi) * math.sin(the) * math.sin(phi) - math.sin(psi) * math.cos(phi)],
        [-math.sin(the),                    math.sin(psi) * math.cos(the),                                                  math.cos(psi) * math.cos(the)]
    ])

    # Why did they divide by cos(the) here???
    iW = np.array([
        [0,             math.sin(psi),                  math.cos(psi)],
        [0,             math.cos(psi) * math.cos(the),  -math.sin(psi) * math.cos(the)],
        [math.cos(the), math.sin(psi) * math.sin(the),  math.cos(psi) * math.sin(the)]
    ]) / math.cos(the)

    # ROTOR MODEL
    T = np.zeros((3, 4))
    Q = np.zeros((3, 4))
    tau = np.zeros((3, 4))
    a1s = np.zeros(4)
    b1s = np.zeros(4)

    for i in [N, E, S, W]:
        Vr = np.cross(o, D[:, i]) + np.linalg.inv(R_Body2World).dot(v)
        # mu = np.linalg.norm(Vr[0:2]) / (abs(w[i]) * quad['r'])        # Magnitude of mu, planar components
        mu = np.linalg.norm(Vr[0:2]) / (abs(w[i]) * quad['r'] + 1e-6)   # Kept getting divide by zero errors
        lc = Vr[2] / (abs(w[i]) * quad['r'])                            # Non-dimensionalised normal inflow
        li = mu                                                         # Non-dimensionalised induced velocity approximation
        alphas = math.atan2(lc, mu)
        j = math.atan2(Vr[1], Vr[0])                                    # Sideslip azimuth relative to e1 (zero over nose)
        J = np.array([                                                  # BBF > mu sideslip rotation matrix               
            [math.cos(j), -math.sin(j)],
            [math.sin(j), math.cos(j)]
        ])

        # Flapping
        beta = np.array([                                           # Longitudal flapping 
            ((8 / 3 * quad['theta0'] + 2 * quad['theta1']) - 2 * lc) / (1 / mu - mu / 2),
            0
        ])
        beta = J.T.dot(beta)                                        # sign(w) * (4/3)*((Ct/sigma)*(2*mu*gamma/3/a)/(1+3*e/2/r) + li)/(1+mu^2/2)]; %Lattitudinal flapping (note sign)
        a1s[i] = beta[0] - 16 / quad['gamma'] / abs(w[i]) * o[1]    # Rotate the beta flapping angles to longitudinal and lateral coordinates.
        b1s[i] = beta[1] - 16 / quad['gamma'] / abs(w[i]) * o[0]
        
        # Fprces and torques
        T[:, i] = quad['Ct'] * quad['rho'] * quad['A'] * quad['r'] ** 2 * w[i] ** 2 * np.array(
            [-math.cos(b1s[i]) * math.sin(a1s[i]), math.sin(b1s[i]), -math.cos(a1s[i]) * math.cos(b1s[i])]
            )                                                                                       # Rotor thrust, linearised angle approximations
        Q[:, i] = -quad['Cq'] * quad['rho'] * quad['A'] * quad['r'] ** 3 * w[i] * abs(w[i]) * e3    # Rotor drag torque - note that this preserves w(i) direction sign
        tau[:, i] = np.cross(D[:, i], T[:, i])                                                      # Torque due to rotor thrust

    # RIGID BODY DYNAMIC MODEL
    dz = v
    dn = iW.dot(o)

    dv = quad['g'] * e3 + R_Body2World.dot(1 / quad['M'] * np.sum(T, axis=1))
    do = np.linalg.inv(quad['J']).dot(-np.cross(o, quad['J'].dot(o)) + np.sum(tau, axis=1) + np.sum(Q, axis=1))     # Row sum of torques

    sys = np.concatenate((dz, dn, dv, do))
    return sys

def modelOutputs(x):
    """
    Calculate the output vector for this timestep. Inherited from the Rolling Spider software package. Again, if it ain't broke, don't fix it.
    
    Args:
        x: The current state of the quadrotor.
    
    Returns:
        sys: The output vector for this timestep.
    """

    # CRASH DETECTION
    if x[2] > 0:
        x[2] = 0
        if x[5] > 0:
            x[5] = 0

    n = x[3:6]    # RPY angles
    phi = n[0]    # yaw
    the = n[1]    # pitch
    psi = n[2]    # roll

    # R_phi @ R_the @ R_psi
    R_Body2World = np.array([
        [math.cos(the) * math.cos(phi), math.sin(psi) * math.sin(the) * math.cos(phi) - math.cos(psi) * math.sin(phi), math.cos(psi) * math.sin(the) * math.cos(phi) + math.sin(psi) * math.sin(phi)],
        [math.cos(the) * math.sin(phi), math.sin(psi) * math.sin(the) * math.sin(phi) + math.cos(psi) * math.cos(phi), math.cos(psi) * math.sin(the) * math.sin(phi) - math.sin(psi) * math.cos(phi)],
        [-math.sin(the), math.sin(psi) * math.cos(the), math.cos(psi) * math.cos(the)]
    ])

    iW = np.array([                             # Inverted Wronskian
        [0, math.sin(psi), math.cos(psi)],
        [0, math.cos(psi) * math.cos(the), -math.sin(psi) * math.cos(the)],
        [math.cos(the), math.sin(psi) * math.sin(the), math.cos(psi) * math.sin(the)]
    ]) / math.cos(the)

    sys = np.concatenate((
        x[0:6],                                     # Output global pos and euler angles
        np.linalg.inv(R_Body2World).dot(x[6:9]),    # Translational velocity mapped to body frame
        x[9:12]                                     # Body rates pqr
    ))

    return sys


TEST_HARNESS_MODE = True

if TEST_HARNESS_MODE:

    import matplotlib.pyplot as plt

    dt = 0.01
    t = 0
    x = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    u = np.array([4000, 4000, 4000, 4000])

    quad = QuadrotorPhysicalConstants.quad
    Quadrotor = QuadrotorModel(quad, dt)

    t_hist = []
    x_hist = []
    u_hist = []

    for i in range(500):
        t_hist.append(t)
        x_hist.append(x)
        u_hist.append(u)

        x = Quadrotor.update(t, x, u)
        t += dt

    x_hist = np.array(x_hist)
    u_hist = np.array(u_hist)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_hist[:, 0], x_hist[:, 1], abs(x_hist[:, 2]))
    ax.set_title('Position')
    ax.set_xlabel('North [m]')
    ax.set_ylabel('East [m]')
    ax.set_zlabel('Down [m]')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    plt.grid()

    plt.figure()
    plt.plot(t_hist, x_hist[:, 3:6])
    plt.title('Attitude')
    plt.legend(['Yaw', 'Pitch', 'Roll'])
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [rad]')
    plt.grid()

    plt.show()