"""
Author:
"""

import numpy as np
from . import QuadrotorModel
import matplotlib.pyplot as plt

takeoff_flag = False # ofc there's a takeoff flag

# So this should take in reference values and state estimations (truth for us)
# and output
class FlightControllerSystem():

    # we need
    # - reference
    # - truth
    def __init__(self, x, ref, dT = 0.01, Scope = False):

        self.dT = dT
        
        self.position_state = x[0:3]
        self.attitude_state = x[3:6]
        self.velocity_state = x[6:9]
        self.angular_velocity_state = x[9:12]

        self.position_ref = ref[0:3]
        self.attitude_ref = ref[3:6]
        self.velocity_ref = ref[6:9]
        self.angular_velocity_ref = ref[9:12]

        self.Scope = Scope
        if self.Scope:
            self.control_hist = []
            self.state_hist = []
            self.ref_hist = []
            self.thrust_hist = []

    def YawController(self):

        yaw_ref = self.attitude_ref[0]
        yaw_state = self.attitude_state[0]
        yaw_dot_state = self.angular_velocity_state[0]


        P_yaw = 0.004           # unhard code these
        D_yaw = 0.3 * 0.004

        tau_yaw = P_yaw * (yaw_ref - yaw_state) - D_yaw * yaw_dot_state

        print("Yaw", yaw_ref)
        print("Yaw State", yaw_state)
        print("Yaw Dot State", yaw_dot_state)
        print("Tau Yaw", tau_yaw)
        print()

        return tau_yaw

    def Rotate_XY2Ref(self):

        yaw = self.attitude_state[2]
        XY_ref = self.position_ref[0:2]
        XY_state = self.position_state[0:2]
        XY_dot_state = self.velocity_state[0:2]

        R = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])

        P_xy = np.array([-0.24, 0.24])
        D_xy = np.array([0.1, -0.1])

        # Saturatin
        temp = R @ (XY_ref - XY_state)
        temp[0] = np.clip(temp[0], -3, 3)
        temp[1] = np.clip(temp[1], -3, 3)

        pitch_roll_CMD = P_xy * temp - D_xy * XY_dot_state

        return pitch_roll_CMD
    
    def GravityFeedforwardEquilibriumThrust(self):
        
        Z_ref = self.position_ref[2]
        Z_state = self.position_state[2]
        Z_dot_state = self.velocity_state[2]

        P_z = 0.8
        D_z = 0.3
        totalThrustMaxRelative = 0.920000000000000
        motorsThrustPerMotorMax = 0.326642213351703

        if takeoff_flag == True:
            pass # implement takeoff
        else:

            # Saturate
            temp = P_z * (Z_ref - Z_state) - D_z * Z_dot_state
            attitude_CMD = np.clip(temp, -4*totalThrustMaxRelative*motorsThrustPerMotorMax,
                                    4*totalThrustMaxRelative*motorsThrustPerMotorMax)
            
        return attitude_CMD
    
    def AttitudeController(self):

        attitude_ref = np.array([self.attitude_ref[0], self.attitude_ref[1]])
        attitude_state = np.array([self.attitude_state[0], self.attitude_state[1]])
        attitude_dot_state = np.array([self.angular_velocity_state[0], self.angular_velocity_state[1]])

        pitch_roll_error = attitude_ref - attitude_state

        P_pr = np.array([0.013, 0.01])
        I_pr = 0.01
        D_pr = np.array([0.002, 0.0028])

        integralComponent = np.zeros(2) # TODO: implement integral component

        P_pr = P_pr * pitch_roll_error - D_pr * attitude_dot_state + I_pr * integralComponent

        tau_pitch = P_pr[0]
        tau_roll = P_pr[1]

        return tau_pitch, tau_roll
    
    def ControlMixer(self):
        tau_yaw = self.YawController()

        pitch_roll_CMD = self.Rotate_XY2Ref()
        self.attitude_ref = np.array([pitch_roll_CMD[0], pitch_roll_CMD[1], self.attitude_ref[2]])

        attitude_CMD = self.GravityFeedforwardEquilibriumThrust()

        tau_pitch, tau_roll = self.AttitudeController()

        Q2TS = np.array([ # Torque total thrust to thrust per motor
            [0.250000000000000, 103.573625306767,   -5.66591972104606,  -5.66591972104606   ],
            [0.250000000000000, -103.573625306767,  -5.66591972104606,  5.66591972104606    ],
            [0.250000000000000, 103.573625306767,   5.66591972104606,   5.66591972104606    ],
            [0.250000000000000, -103.573625306767,  5.66591972104606,   -5.66591972104606   ]
        ])

        if self.Scope:
            self.control_hist.append([attitude_CMD, tau_pitch, tau_roll, tau_yaw])

        thrusts = Q2TS @ np.array([attitude_CMD, tau_pitch, tau_roll, tau_yaw])

        return thrusts

    def thrustsToMotorCommands(self):

        thrustToMotorCommand = 1530.7
        maxLimit = 500
        minLimit = 10
        motorDirections = np.array([1, -1, 1, -1])

        thrusts = self.ControlMixer()

        motor_CMD = thrusts * -1 * thrustToMotorCommand
        motor_CMD = np.clip(motor_CMD, minLimit, maxLimit)
        motor_CMD = motor_CMD * motorDirections

        if self.Scope:
            self.thrust_hist.append(motor_CMD)

        return motor_CMD

    def update(self, x, ref):

        self.position_state = x[0:3]
        self.velocity_state = x[3:6]
        self.attitude_state = x[6:9]
        self.angular_velocity_state = x[9:12]

        self.position_ref = ref[0:3]
        self.velocity_ref = ref[3:6]
        self.attitude_ref = ref[6:9]
        self.angular_velocity_ref = ref[9:12]

        motor_CMD = self.thrustsToMotorCommands()

        pos_ref = self.position_ref
        orient_ref = self.attitude_ref
        pose_refout = np.concatenate((pos_ref, orient_ref))

        if self.Scope:
            self.state_hist.append(x)
            self.ref_hist.append(ref)

        return pose_refout, motor_CMD
    
    def getHist(self):
        return self.control_hist, self.state_hist, self.ref_hist, self.thrust_hist
    
    def plotScope(self, pos_state = True, pos_ref = True, orientation_state = True, orientation_ref = True,
                  control = True, thrust = True):
        
        # Unpack data

        control_hist, state_hist, ref_hist, thrust_hist = self.getHist()
        control_hist = np.array(control_hist)
        state_hist = np.array(state_hist)
        ref_hist = np.array(ref_hist)
        thrust_hist = np.array(thrust_hist)

        x, y, z = state_hist[:, 0], state_hist[:, 1], state_hist[:, 2]
        yaw, pitch, roll = state_hist[:, 3], state_hist[:, 4], state_hist[:, 5]
        
        x_ref, y_ref, z_ref = ref_hist[:, 0], ref_hist[:, 1], ref_hist[:, 2]
        yaw_ref, pitch_ref, roll_ref = ref_hist[:, 3], ref_hist[:, 4], ref_hist[:, 5]

        u1, u2, u3, u4 = control_hist[:, 0], control_hist[:, 1], control_hist[:, 2], control_hist[:, 3]
        motor1, motor2, motor3, motor4 = thrust_hist[:, 0], thrust_hist[:, 1], thrust_hist[:, 2], thrust_hist[:, 3]

        t = np.linspace(0, self.dT * len(x), len(x))

        if pos_state:
            # Plot x, y, z over time
            plt.figure()
            plt.plot(t, x, label='x')
            plt.plot(t, y, label='y')
            plt.plot(t, z, label='z')
            plt.xlabel('Time')
            plt.ylabel('Position')
            plt.legend()
            plt.title('Position over Time')

        if pos_ref:
            # Plot x_ref, y_ref, z_ref over time
            plt.figure()
            plt.plot(t, x_ref, label='x_ref')
            plt.plot(t, y_ref, label='y_ref')
            plt.plot(t, z_ref, label='z_ref')
            plt.xlabel('Time')
            plt.ylabel('Reference Position')
            plt.legend()
            plt.title('Reference Position over Time')

        if orientation_state:
            # Plot yaw, pitch, roll over time
            plt.figure()
            plt.plot(t, yaw, label='yaw')
            plt.plot(t, pitch, label='pitch')
            plt.plot(t, roll, label='roll')
            plt.xlabel('Time')
            plt.ylabel('Orientation')
            plt.legend()
            plt.title('Orientation over Time')
        
        if orientation_ref:
            # Plot yaw_ref, pitch_ref, roll_ref over time
            plt.figure()
            plt.plot(t, yaw_ref, label='yaw_ref')
            plt.plot(t, pitch_ref, label='pitch_ref')
            plt.plot(t, roll_ref, label='roll_ref')
            plt.xlabel('Time')
            plt.ylabel('Reference Orientation')
            plt.legend()
            plt.title('Reference Orientation over Time')

        if control:
            # Plot u1, u2, u3, u4 over time
            plt.figure()
            plt.plot(t, u1, label='u1')
            plt.plot(t, u2, label='u2')
            plt.plot(t, u3, label='u3')
            plt.plot(t, u4, label='u4')
            plt.xlabel('Time')
            plt.ylabel('Control Inputs')
            plt.legend()
            plt.title('Control Inputs over Time')

        if thrust:
            # Plot motor1, motor2, motor3, motor4 over time
            plt.figure()
            plt.plot(motor1, label='motor1')
            plt.plot(motor2, label='motor2')
            plt.plot(motor3, label='motor3')
            plt.plot(motor4, label='motor4')
            plt.xlabel('Time')
            plt.ylabel('Motor Commands')
            plt.legend()
            plt.title('Motor Commands over Time')
        
        plt.show()