"""
Author: Eric Vetha (evetha@ucsc.edu)


"""

import math
import numpy as np

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
        sys = 1
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

TEST_HARNESS_MODE = True

if TEST_HARNESS_MODE:

    sys, x0, str, ts = quadrotor_dynamics(1, 1, 1, 0, 1)
    print(sys, x0, str, ts)