"""
Author: Eric Vetha (evetha@ucsc.edu)

WindModel.py implements the Dryden gust model and allows for an update of the wind at every time step when driven
by white noise. The constants for the wind model are embedded in the init function, and an init flag tells the __init__
function which set of paramters to grab.

All of the constants that are used in the Dryden Wind Gust Models. These are detailed on Beard Chapter 4.4. The
Dryden models are typically implemented assuming a constant nominal airspeed (taken as initial speed from physical
constants). The parameters for the Dryden gust model are defined in MIL-F-8785C. See ECE163_DrydenWindModel
writeup for details.
"""

import math
import random
from ..Containers import States
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC
from ..Containers import Inputs
import numpy as np

class WindModel:
    
    def __init__(self, dT=0.01, Va=25.0, drydenParameters=Inputs.drydenParameters(Lu=0.0, Lv=0.0, Lw=0.0,
        sigmau=0.0, sigmav=0.0, sigmaw=0.0)):
        """
        Function to initialize the wind model code. Will load the appropriate constants that parameterize the wind gusts from
        the Dryden gust model. Creates the discrete transfer functions for the gust models that are used to update
        the local wind gusts in the wind frame. These are added to the inertial wind (Wn, We, Wd) that are simply
        constants. Discrete models are held in self and used in the Update function.

        To turn off gusts completely, use the DrydenNoGusts parameters.
        
        :param dT: Time step [s] for numerical integration of wind.
        :param Va: Nominal flight speed [m/s].
        :param drydenParameters: Dryden parameters for the gust model.
        :return: None
        """

        self.dT = dT
        self.Va = Va
        self.drydenParameters = drydenParameters
        self.x_u, self.x_v, self.x_w = [[0.0]], [[0.0], [0.0]], [[0.0], [0.0]]
        self.sigma, self.gamma, self.H = 0.0, 0.0, 0.0
        self.u, self.v, self.w = 0.0, 0.0, 0.0
        self.CreateDrydenTransferFns(dT, Va, drydenParameters)
        self.windState = States.windState(Wn=0.0, We=0.0, Wd=0.0, Wu=0.0, Wv=0.0, Ww=0.0)
        
    
    def CreateDrydenTransferFns(self, dT, Va, drydenParameters):
        """
        Function creates the Dryden transfer functions in discrete form. These are used in generating the gust
        models for wind gusts (in wind frame). If the input is DrydenNoWind, then set Phi_[u,v,w] to Identity,
        Gamma_{u,v,w]tozero, and H_[u,v,w] to a columnofones. If input Vais less than or 0, raise an arithmetic
        error since your wind models are undefined for zero or negative airspeeds.
        
        :param dT: Time step [s].
        :param Va: Nominal flight speed [m/s].
        :param drydenParameters: Dryden Wing Gust Model.
        :return: None, gust models are created internally.
        """

        L_u, L_v, L_w = drydenParameters.Lu, drydenParameters.Lv, drydenParameters.Lw
        sigma_u, sigma_v, sigma_w = drydenParameters.sigmau, drydenParameters.sigmav, drydenParameters.sigmaw

        if L_u != 0.0:
            Phi_u = np.matrix([[np.exp(-Va/L_u*dT)]])
            Gamma_u = np.matrix([[L_u/Va*(1-np.exp(-Va/L_u*dT))]])
            H_u = np.matrix([[sigma_u*np.sqrt(2*Va/(L_u*np.pi))]])
        else:
            Phi_u = np.matrix([[1.0]])
            Gamma_u = np.matrix([[0.0]])
            H_u = np.matrix([[1.0]])

        if L_v != 0.0:
            Phi_v = np.exp(-Va/L_v*dT) * np.matrix([
                [1 - Va/L_v*dT, -(Va/L_v)**2 * dT],
                [dT, 1 + Va/L_v*dT]
            ])
            Gamma_v = np.exp(-Va/L_v*dT) * np.matrix([
                [dT],
                [(L_v/Va)**2 * (np.exp(Va/L_v*dT) - 1) - L_v/Va*dT]
            ])
            H_v = sigma_v * np.sqrt(3*Va/(np.pi*L_v)) * np.matrix([
                [1, Va/(np.sqrt(3)*L_v)]
            ])
        else:
            Phi_v = np.matrix([[1.0, 0.0], [0.0, 1.0]])
            Gamma_v = np.matrix([[0.0], [0.0]])
            H_v = np.matrix([[1.0, 1.0]])

        if L_w != 0.0:
            Phi_w = np.exp(-Va/L_w*dT) * np.matrix([
                [1 - Va/L_w*dT, -(Va/L_w)**2 * dT],
                [dT, 1 + Va/L_w*dT]
            ])
            Gamma_w = np.exp(-Va/L_w*dT) * np.matrix([
                [dT],
                [(L_w/Va)**2 * (np.exp(Va/L_w*dT) - 1) - L_w/Va*dT]
            ])
            H_w = sigma_w * np.sqrt(3*Va/(np.pi*L_w)) * np.matrix([
                [1, Va/(np.sqrt(3)*L_w)]
            ])
        else:
            Phi_w = np.matrix([[1.0, 0.0], [0.0, 1.0]])
            Gamma_w = np.matrix([[0.0], [0.0]])
            H_w = np.matrix([[1.0, 1.0]])

        self.Phi_u, self.Phi_v, self.Phi_w = Phi_u.tolist(), Phi_v.tolist(), Phi_w.tolist()
        self.Gamma_u, self.Gamma_v, self.Gamma_w = Gamma_u.tolist(), Gamma_v.tolist(), Gamma_w.tolist()
        self.H_u, self.H_v, self.H_w = H_u.tolist(), H_v.tolist(), H_w.tolist()
    
    def Update(self, uu=None, uv=None, uw=None):
        """
        def Update(self, uu=None, uv=None, uw=None): Function that updates the wind gusts and inserts them
        back into the .Wind portion of self. This is done by running white noise [Gaussian(0,1)] through the
        coloring filters of the Dryden Wind Gust model.
        
        :param uu: Optional input to Hu(s), defaults to None.
        :param uv: Optional input to Hv(s), defaults to None.
        :param uw: Optional input to Hw(s), defaults to None.
        :return: None, gust values are updated internally.
        """

        if uu is None:
            uu = random.gauss(0, 1)
        if uv is None:
            uv = random.gauss(0, 1)
        if uw is None:
            uw = random.gauss(0, 1)

        self.CreateDrydenTransferFns(self.dT, self.Va, self.drydenParameters)

        x_minus_u, x_minus_v, x_minus_w = self.x_u, self.x_v, self.x_w
        x_plus_u = np.matrix(self.Phi_u) @ np.matrix(x_minus_u) + np.matrix(self.Gamma_u) * np.matrix(uu)
        x_plus_v = np.matrix(self.Phi_v) @ np.matrix(x_minus_v) + np.matrix(self.Gamma_v) * np.matrix(uv)
        x_plus_w = np.matrix(self.Phi_w) @ np.matrix(x_minus_w) + np.matrix(self.Gamma_w) * np.matrix(uw)

        W_u = np.matrix(self.H_u) * np.matrix(x_plus_u)
        W_v = np.matrix(self.H_v) * np.matrix(x_plus_v)
        W_w = np.matrix(self.H_w) * np.matrix(x_plus_w)

        self.windState.Wu = W_u[0, 0]
        self.windState.Wv = W_v[0, 0]
        self.windState.Ww = W_w[0, 0]

        self.x_u, self.x_v, self.x_w = x_plus_u.tolist(), x_plus_v.tolist(), x_plus_w.tolist()
    
    def getDrydenTransferFns(self):
        """
        Wrapper function to return the internals of the Dryden Transfer function in order to be able to test the code
        without requiring consistent internal names. Returns the discretized version of the Drydem gust model
        as outlined in the ECE163_DrydenWindModel handout (Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v,
        Phi_w, Gamma_w, H_w).
        
        :return: Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w.
        """

        return self.Phi_u, self.Gamma_u, self.H_u, self.Phi_v, self.Gamma_v, self.H_v, self.Phi_w, self.Gamma_w, self.H_w
        
    
    def getWind(self):
        """
        Wrapper function to return the wind state from the module
        
        :return: WindState class.
        """

        return self.windState
        
    
    def reset(self):
        """
        Wrapper function that resets the wind model code (but does not reset the model parameters). This will
        reset both steady and gust winds to zero, and re-set the internal states of the stochastic Dyden wind model
        to zero. To change the model transfer functions you need to use CreateDrydenTranferFns().
        
        :return: None
        """

        self.windState = States.windState(Wn=0.0, We=0.0, Wd=0.0, Wu=0.0, Wv=0.0, Ww=0.0)
        
    
    def setWind(self, windState):
        """
        Wrapper function that allows for injecting constant wind and gust values into the class :param windState:
        class from vehicleStates with inertial constant wind and wind frame gusts
        
        :param windState: WindState class containing inertial constant wind and wind frame gusts.
        :return: None
        """

        self.windState = windState
        
    
    def setWindModelParameters(self, Wn=0.0, We=0.0, Wd=0.0, drydenParameters=Inputs.drydenParameters(Lu=0.0,
        Lv=0.0, Lw=0.0, sigmau=0.0, sigmav=0.0, sigmaw=0.0)):
        """
        Wrapper function that will inject parameters into the wind model. This class models wind according to
        the Dryden Wind Model, where winds are a static component plus stocahstically derived gusts.
        
        :param Wn: Steady wind in the inertial North direction [m/s].
        :param We: Steady wind in the inertial East direction [m/s].
        :param Wd: Steady wind in the inertial Down direction [m/s], usually zero.
        :param drydenParameters: Dryden parameters to model gusts.
        :return: None
        """

        self.windState.Wn, self.windState.We, self.windState.Wd = Wn, We, Wd
        self.drydenParameters = drydenParameters
        self.CreateDrydenTransferFns(self.dT, self.Va, drydenParameters)