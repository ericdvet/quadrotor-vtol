"""
Author: Eric Vetha (evetha@ucsc.edu)


"""

import math
import numpy as np
import VTOL_VehiclePhysicalConstants

def quadrotor_dynamics(t, x, u, flag, quad):
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
    elif flag == 3:
        # Calculate outputs
        sys = 3
    else:
        raise NotImplementedError
    
    return sys, x0, str, ts 

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

TEST_HARNESS_MODE = True

if TEST_HARNESS_MODE:

    import matplotlib as plt

    x = np.array([
        0, 0, -0.046,   
        0, 0, 0,     
        0, 0, 0,      
        0, 0, 0        
    ])
    u = np.array([0, 0, 0, 0])  
    dt = 0.01
    t = 0

    for i in range(1000):
        u = np.array([4000, 4000, 4000, 4000])

        sys = modelDerivatives(t, x, u, VTOL_VehiclePhysicalConstants.quad)

        x = x + sys * dt
        t += dt

        print(f"Time: {t:.2f}, Position: {x[0:3]}")
