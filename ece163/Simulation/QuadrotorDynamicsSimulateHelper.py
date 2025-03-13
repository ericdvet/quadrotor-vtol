from . import Simulate
from ..Quadrotor import QuadrotorModel
from ..Quadrotor import QuadrotorPhysicalConstants as quad

import numpy as np

class QuadrotorDynamicsSimulateHelper(Simulate.Simulate):
	def __init__(self):
		super().__init__()
		self.inputNames.extend(['Throttle', 'Aileron', 'Elevator', 'Rudder'])
		# self.inputNames.extend(['Motor 1', 'Motor 2', 'Motor 3', 'Motor 4'])
		
		self.dT = 0.01
		self.x = np.array([0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		
		self.underlyingModel = QuadrotorModel.QuadrotorModel(quad.quad, x0=self.x, dT = self.dT)

		# self.variableList.append((self.underlyingModel.getForcesMoments, 'ForceMoments',
		# 							['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']))

		self.variableList.append((self.underlyingModel.getVehicleState, 'state',
									['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'Va', 'alpha', 'beta']))
		# self.dT = 1/50


	def getVehicleState(self):
		return self.underlyingModel.getVehicleState()

	def takeStep(self, controlInput):
		self.time += self.dT
		self.x = self.underlyingModel.update(self.x, controlInput)
		self.recordData([controlInput[0], controlInput[1], controlInput[2], controlInput[3]])
		return

	def reset(self):
		self.time = 0
		x = np.array([0, 0, -1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		self.underlyingModel = QuadrotorModel.QuadrotorModel(quad.quad, x0=x, dT = self.dT)
		self.takenData.clear()