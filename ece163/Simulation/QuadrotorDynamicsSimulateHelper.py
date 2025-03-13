from . import Simulate
from ..Quadrotor import QuadrotorModel
from ..Quadrotor import QuadrotorPhysicalConstants as quad

import numpy as np

class QuadrotorDynamicsSimulateHelper(Simulate.Simulate):
	def __init__(self, mode=0):
		super().__init__()
		self.inputNames.extend(['Throttle', 'Aileron', 'Elevator', 'Rudder'])
		# self.inputNames.extend(['Motor 1', 'Motor 2', 'Motor 3', 'Motor 4'])
		
		self.dT = 0.01
		self.mode = mode
		
		if self.mode == 0 or self.mode == 2: # free mode or hover mode
			self.x = np.array([0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		elif self.mode == 1: # take off mode
			self.x = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		else:
			exit("Invalid mode")

		self.crashFlag = False
		
		self.underlyingModel = QuadrotorModel.QuadrotorModel(quad.quad, x0=self.x, dT = self.dT)

		# self.variableList.append((self.underlyingModel.getForcesMoments, 'ForceMoments',
		# 							['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']))

		self.variableList.append((self.underlyingModel.getVehicleState, 'state',
									['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'Va', 'alpha', 'beta']))
		# self.dT = 1/50


	def getVehicleState(self):
		return self.underlyingModel.getVehicleState()

	def takeStep(self, controlInput):
		if self.crashFlag == False:
			self.time += self.dT
			if self.mode == 0:  # free mode
				try:
					self.x = self.underlyingModel.update(self.x, controlInput)
				except:
					self.crashFlag = True
					print("Quadrotor is pitched over. Dynamics model fails past this point.")
				self.recordData([controlInput[0], controlInput[1], controlInput[2], controlInput[3]])
			elif self.mode == 1:  # take off
				try:
					controlInput = np.array([3000, -3000, 3000, -3000])
					self.x = self.underlyingModel.update(self.x, controlInput)
				except:
					self.crashFlag = True
					print("Quadrotor is pitched over. Dynamics model fails past this point.")
				self.recordData([controlInput[0], controlInput[1], controlInput[2], controlInput[3]])
			elif self.mode == 2:  # hover mode
				try:
					dz = self.x[8]
					controlInput = np.array([1000, -1000, 1000, -1000])
					if dz > 0:
						controlInput = np.array([2500, -2500, 2500, -2500])
					self.x = self.underlyingModel.update(self.x, controlInput)
				except:
					self.crashFlag = True
					print("Quadrotor is pitched over. Dynamics model fails past this point.")
				self.recordData([controlInput[0], controlInput[1], controlInput[2], controlInput[3]])
		else:
			return
		return

	def reset(self):
		self.time = 0
		self.crashFlag = False
		
		if self.mode == 0 or self.mode == 2:
			self.x = np.array([0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		elif self.mode == 1:
			self.x = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		
		self.underlyingModel = QuadrotorModel.QuadrotorModel(quad.quad, x0=self.x, dT = self.dT)
		self.takenData.clear()