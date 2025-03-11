"""
Author: Eric Vetha (evetha@ucsc.edu)

Description: This module contains the QuadrotorModel class, which is a simulation of idealized X-4 Flyer II flight dynamics based upon Pounds et al. (2010). Much of the logic is inhereted from the Rolling Spider software package version 6.0.0. 
"""

import math
import numpy as np
import VTOL_VehiclePhysicalConstants
import States

class QuadrotorModel:
    def __init__(self, quad, dT = 0.01):
        """
        Initialization of the internal classes which are used to track the vehicle aerodynamics and dynamics.

        Args:
            quad (dict): A dictionary of quadrotor physical constants. This is defined in VTOL_VehiclePhysicalConstants.py.
            dT (float): The time step for the simulation

        Returns:
            None
        """

        self.quad = quad
        self.dT = dT

        self.sys, self.x0, self.str, self.ts = self.quadrotor_dynamics(0, np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0]), np.array([0, 0, 0, 0]), 0, self.quad)

        self.x = self.x0

    def update(self, t, x, u):
        """
        Function that uses the current internal state (x) and motor commands (u) to update the state of the quadrotor. 
        
        Args:
            t: No clue
            x: Also no clue
            u (np.array 1x4): NSWE motor commands
        
        Returns:
            x: The updated state of the quadrotor
        """

        self.sys, self.x0, self.str, self.ts = self.quadrotor_dynamics(t, x, u, 1, self.quad)
        x = x + self.sys * self.dT
        self.sys, self.x0, self.str, self.ts = self.quadrotor_dynamics(t, x, u, 3, self.quad)
        self.x = x
        return x


    def quadrotor_dynamics(self, t, x, u, flag, quad):
        """
        A simulation of idealized X-4 Flyer II flight dynamics based upon Pounds et al. (2010).
        
        Args:
            t: No clue
            x: Also no clue
            u (np.array 1x4): NSWE motor commands
        """

        """
        init = [z1, z2, z3, n1, n2, n3, v1, v2, v3, o1, o2, o3]

        z0      Position initial conditions             1x3
        n0      Angular position initial conditions     1x3
        v0      Velocity initial conditions             1x3
        o0      Angular velocity initial conditions     1x3
        """
        init = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        if flag == 0:
            sys, x0, str, ts = initializeModel(init, quad)
        elif flag == 1:
            # Calculate derivatives
            sys = modelDerivatives(t, x, u, quad)
            x0 = []
            str = []
            ts = []
        elif flag == 3:
            # Calculate outputs
            sys = modelOutputs(t, x, quad)
            x0 = []
            str = []
            ts = []
        else:
            raise NotImplementedError
        
        return sys, x0, str, ts 
    
    def wrapper_vehicleState(self):
        """
        Converts the quadrotor state vector to a vehicleState object for use in the simulation.
        """

        vehicleState = States.vehicleState(pn = self.x[0], pe = self.x[1], pd = self.x[2], 
                                           u = self.x[6], v = self.x[7], w = self.x[8], 
                                           yaw = self.x[3], pitch = self.x[4], roll = self.x[5], 
                                           p = self.x[9], q = self.x[10], r = self.x[11])
        return vehicleState

def initializeModel(init, quad):

    sys = 0
    x0 = init
    str = np.matrix([[]])
    ts = np.array([0, 0])
    
    return sys, x0, str, ts

def modelDerivatives(t, x, u, quad):
    
    # CONSTANTS
    # Cardinal Direction Indices
    N = 0  # North
    E = 1  # East
    S = 2  # South
    W = 3  # West

    d = math.sqrt(2) / 2 * quad['d']

    D = np.array([                  # Rotor hub displacements
        [d, -d, quad['h']],
        [d, d, quad['h']],
        [-d, d, quad['h']],
        [-d, -d, quad['h']]
    ]).T

    # Body-fixed frame references
    e1 = np.array([1, 0, 0])
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

    R_Body2World = np.array([
        [math.cos(the) * math.cos(phi),     math.sin(psi) * math.sin(the) * math.cos(phi) - math.cos(psi) * math.sin(phi),  math.cos(psi) * math.sin(the) * math.cos(phi) + math.sin(psi) * math.sin(phi)],
        [math.cos(the) * math.sin(phi),     math.sin(psi) * math.sin(the) * math.sin(phi) + math.cos(psi) * math.cos(phi),  math.cos(psi) * math.sin(the) * math.sin(phi) - math.sin(psi) * math.cos(phi)],
        [-math.sin(the),                    math.sin(psi) * math.cos(the),                                                  math.cos(psi) * math.cos(the)]
    ])

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
        mu = np.linalg.norm(Vr[0:2]) / (abs(w[i]) * quad['r'] + 1e-6) # mu = np.linalg.norm(Vr[0:2]) / (abs(w[i]) * quad['r'])
        lc = Vr[2] / (abs(w[i]) * quad['r'])
        li = mu
        alphas = math.atan2(lc, mu)
        j = math.atan2(Vr[1], Vr[0])
        J = np.array([
            [math.cos(j), -math.sin(j)],
            [math.sin(j), math.cos(j)]
        ])

        beta = np.array([
            ((8 / 3 * quad['theta0'] + 2 * quad['theta1']) - 2 * lc) / (1 / mu - mu / 2),
            0
        ])
        beta = J.T.dot(beta)
        a1s[i] = beta[0] - 16 / quad['gamma'] / abs(w[i]) * o[1]
        b1s[i] = beta[1] - 16 / quad['gamma'] / abs(w[i]) * o[0]

        T[:, i] = quad['Ct'] * quad['rho'] * quad['A'] * quad['r'] ** 2 * w[i] ** 2 * np.array([-math.cos(b1s[i]) * math.sin(a1s[i]), math.sin(b1s[i]), -math.cos(a1s[i]) * math.cos(b1s[i])])
        Q[:, i] = -quad['Cq'] * quad['rho'] * quad['A'] * quad['r'] ** 3 * w[i] * abs(w[i]) * e3
        tau[:, i] = np.cross(D[:, i], T[:, i])

    dz = v
    dn = iW.dot(o)
    dv = quad['g'] * e3 + R_Body2World.dot(1 / quad['M'] * np.sum(T, axis=1))
    do = np.linalg.inv(quad['J']).dot(-np.cross(o, quad['J'].dot(o)) + np.sum(tau, axis=1) + np.sum(Q, axis=1))

    if np.isnan(do).any():
        raise ValueError('system rotating NaN!')
        do = np.zeros_like(do)

    sys = np.concatenate((dz, dn, dv, do))
    return sys

def modelOutputs(t, x, quad):
    # CRASH DETECTION
    if x[2] > 0:
        x[2] = 0
        if x[5] > 0:
            x[5] = 0

    # TELEMETRY
    if quad['verbose']:
        print(f"{t:.3f}\t{x}")

    # compute output vector as a function of state vector
    #   z      Position                         3x1   (x,y,z)
    #   v      Velocity                         3x1   (xd,yd,zd)
    #   n      Attitude                         3x1   (Y,P,R)
    #   o      Angular velocity                 3x1   (Yd,Pd,Rd)

    n = x[3:6]    # RPY angles
    phi = n[0]    # yaw
    the = n[1]    # pitch
    psi = n[2]    # roll

    # rotz(phi)*roty(the)*rotx(psi)
    R_Body2World = np.array([
        [math.cos(the) * math.cos(phi), math.sin(psi) * math.sin(the) * math.cos(phi) - math.cos(psi) * math.sin(phi), math.cos(psi) * math.sin(the) * math.cos(phi) + math.sin(psi) * math.sin(phi)],
        [math.cos(the) * math.sin(phi), math.sin(psi) * math.sin(the) * math.sin(phi) + math.cos(psi) * math.cos(phi), math.cos(psi) * math.sin(the) * math.sin(phi) - math.sin(psi) * math.cos(phi)],
        [-math.sin(the), math.sin(psi) * math.cos(the), math.cos(psi) * math.cos(the)]
    ])

    iW = np.array([
        [0, math.sin(psi), math.cos(psi)],
        [0, math.cos(psi) * math.cos(the), -math.sin(psi) * math.cos(the)],
        [math.cos(the), math.sin(psi) * math.sin(the), math.cos(psi) * math.sin(the)]
    ]) / math.cos(the)

    # return velocity in the body frame
    sys = np.concatenate((
        x[0:6],                         # output global pos and euler angles
        np.linalg.inv(R_Body2World).dot(x[6:9]),  # translational velocity mapped to body frame
        x[9:12]                         # body rates pqr
    ))

    return sys


TEST_HARNESS_MODE = True

if TEST_HARNESS_MODE:

    import matplotlib.pyplot as plt

    dt = 0.01
    t = 0
    x = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    u = np.array([4000, 4000, 4000, 4000])

    quad = VTOL_VehiclePhysicalConstants.quad
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

    # plt.figure()
    # plt.plot(t_hist, x_hist[:, 6:9])
    # plt.title('Velocity')
    # plt.legend(['North', 'East', 'Down'])
    # plt.xlabel('Time [s]')
    # plt.ylabel('Velocity [m/s]')
    # plt.grid()

    # plt.figure()
    # plt.plot(t_hist, x_hist[:, 9:12])
    # plt.title('Angular Velocity')
    # plt.legend(['Yaw', 'Pitch', 'Roll'])
    # plt.xlabel('Time [s]')
    # plt.ylabel('Angular Velocity [rad/s]')
    # plt.grid()

    # plt.figure()
    # plt.plot(t_hist, u_hist)
    # plt.title('Motor Commands')
    # plt.legend(['North', 'East', 'South', 'West'])
    # plt.xlabel('Time [s]')
    # plt.ylabel('Motor Command [rpm]')
    # plt.grid()

    plt.show()