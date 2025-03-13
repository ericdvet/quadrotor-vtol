"""
Author:
"""

import numpy as np
from . import QuadrotorModel
import matplotlib.pyplot as plt
from . import QuadrotorPhysicalConstants as QPC

takeoff_flag = False # Not using this

# Tuning parameters

# Yaw Controller
P_yaw = 0.004
D_yaw = 0.3 * 0.004

# Altitude Controller
P_z = -0.8
D_z = -0.3
totalThrustMaxRelative = 0.920000000000000
motorsThrustPerMotorMax = 0.326642213351703

# Attitude Controller
P_pr = np.array([0.013, 0.01])
I_pr = 0.01
D_pr = np.array([0.002, 0.0028])
antiWU_Gain = 0.001  

# idk 
P_xy = np.array([-0.24, 0.24])
D_xy = np.array([0.1, -0.1])

# Control Mixer
thrustToMotorCommand = 1530.7
maxLimit = 500
minLimit = 10
motorDirections = np.array([1, -1, 1, -1])

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

        self.prev_pitch_roll_error = np.zeros(2)
        self.prev_integralComponent = np.zeros(2)

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

        tau_yaw = P_yaw * (yaw_ref - yaw_state) - D_yaw * yaw_dot_state

        return tau_yaw

    def Rotate_XY2Ref(self):

        yaw = self.attitude_state[0]
        XY_ref = self.position_ref[0:2]
        XY_state = self.position_state[0:2]
        XY_dot_state = self.velocity_state[0:2]

        R = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])

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

        if takeoff_flag == True:
            pass # implement takeoff
        else:

            # Saturate
            attitude_CMD = P_z * (Z_ref - Z_state) - D_z * Z_dot_state
        attitude_CMD = attitude_CMD - QPC.quad['M'] * QPC.quad['g']
        attitude_CMD = np.clip(attitude_CMD, -4*totalThrustMaxRelative*motorsThrustPerMotorMax,
                                    4*totalThrustMaxRelative*motorsThrustPerMotorMax)
            
        return attitude_CMD
    
    def AttitudeController(self):

        attitude_ref = np.array([self.attitude_ref[1], self.attitude_ref[2]])
        attitude_state = np.array([self.attitude_state[1], self.attitude_state[2]])
        attitude_dot_state = np.array([self.angular_velocity_state[1], self.angular_velocity_state[2]])

        pitch_roll_error = attitude_ref - attitude_state

        integralComponent = self.prev_integralComponent + (pitch_roll_error + self.prev_pitch_roll_error) * self.dT / 2
        integralComponent += antiWU_Gain * (self.prev_integralComponent - self.prev_pitch_roll_error) # iffy about this

        P_pr_out = P_pr * pitch_roll_error - D_pr * attitude_dot_state + I_pr * integralComponent

        tau_pitch = P_pr_out[0]
        tau_roll = P_pr_out[1]

        self.prev_pitch_roll_error = pitch_roll_error
        self.prev_integralComponent = integralComponent

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

        thrusts = self.ControlMixer()

        motor_CMD = thrusts * -1 * thrustToMotorCommand
        motor_CMD = np.clip(motor_CMD, minLimit, maxLimit)
        motor_CMD = motor_CMD * motorDirections

        if self.Scope:
            self.thrust_hist.append(motor_CMD)

        return motor_CMD

    def update(self, x, ref):

        self.position_state = x[0:3]
        self.attitude_state = x[3:6]
        self.velocity_state = x[6:9]
        self.angular_velocity_state = x[9:12]

        self.position_ref = ref[0:3]
        self.attitude_ref = ref[3:6]
        self.velocity_ref = ref[6:9]
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
                  control = True, thrust = True, controllerTuning = True):
        
        # Unpack data

        print("Plotting Scope")

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
            plt.plot(t, motor1, label='motor1')
            plt.plot(t, motor2, label='motor2')
            plt.plot(t, motor3, label='motor3')
            plt.plot(t, motor4, label='motor4')
            plt.xlabel('Time')
            plt.ylabel('Motor Commands')
            plt.legend()
            plt.title('Motor Commands over Time')
        
        if controllerTuning:
            # Plot yaw_ref, yaw, tau_yaw over time
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(t, yaw_ref, label='yaw_ref')
            plt.plot(t, yaw, label='yaw state')
            plt.xlabel('Time')
            plt.ylabel('Yaw')
            plt.legend()
            plt.title('Yaw over Time')

            plt.subplot(2, 1, 2)
            plt.plot(t, u4, label='tau_yaw')
            plt.xlabel('Time')
            plt.ylabel('Tau Yaw')
            plt.legend()
            plt.title('Tau Yaw over Time')

            # Plot pitch_ref, pitch, tau_pitch over time
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(t, pitch_ref, label='pitch_ref')
            plt.plot(t, pitch, label='pitch state')
            plt.xlabel('Time')
            plt.ylabel('Pitch')
            plt.legend()
            plt.title('Pitch over Time')

            plt.subplot(2, 1, 2)
            plt.plot(t, u2, label='tau_pitch')
            plt.xlabel('Time')
            plt.ylabel('Tau Pitch')
            plt.legend()
            plt.title('Tau Pitch over Time')

            # Plot roll_ref, roll, tau_roll over time
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(t, roll_ref, label='roll_ref')
            plt.plot(t, roll, label='roll state')
            plt.xlabel('Time')
            plt.ylabel('Roll')
            plt.legend()
            plt.title('Roll over Time')

            plt.subplot(2, 1, 2)
            plt.plot(t, u3, label='tau_roll')
            plt.xlabel('Time')
            plt.ylabel('Tau Roll')
            plt.legend()
            plt.title('Tau Roll over Time')

            # Plot thrust over time
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(t, z_ref, label='thrust_ref')
            plt.plot(t, z, label='thrust state')
            plt.xlabel('Time')
            plt.ylabel('Roll')
            plt.legend()
            plt.title('thrust over Time')

            plt.subplot(2, 1, 2)
            plt.plot(t, u1, label='thrust')
            plt.xlabel('Time')
            plt.ylabel('thrust')
            plt.legend()
            plt.title('thrust over Time')
        
        plt.show()