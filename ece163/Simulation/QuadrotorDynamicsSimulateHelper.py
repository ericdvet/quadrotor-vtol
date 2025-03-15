from . import Simulate
from ..Quadrotor import QuadrotorModel
from ..Quadrotor import QuadrotorPhysicalConstants as quad

import numpy as np

TAKEOFF = 0
HOVER = 1
LAND = 2

class QuadrotorDynamicsSimulateHelper(Simulate.Simulate):
	def __init__(self, mode=0):
		super().__init__()
		self.inputNames.extend(['Throttle', 'Aileron', 'Elevator', 'Rudder'])
		# self.inputNames.extend(['Motor 1', 'Motor 2', 'Motor 3', 'Motor 4'])
		
		self.dT = 0.01
		self.mode = mode
		
		if self.mode == 0 or self.mode == 2 or self.mode == 3: # free mode or hover mode
			self.x = np.array([0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		elif self.mode == 1: # take off mode
			self.x = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		else:
			exit("Invalid mode")

		self.crashFlag = False
		# Mode 4 SM Stuff
		self.STATE = 0
		
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
			elif self.mode == 3:
				try:
					posz = self.x[2]
					controlInput = np.array([1000, -1000, 1000, -1000])
					if posz < (posz*0.75):
						controlInput = np.array([750, -750, 750, -750]) # drop
					elif posz >= (posz*0.75):
						controlInput = np.array([980, -980, 980, -980]) # slow
					elif posz >= -0.046:
						controlInput = np.array([0, 0, 0, 0])
					self.x = self.underlyingModel.update(self.x, controlInput)
				except:
					self.crashFlag = True
					print("Quadrotor is pitched over. Dynamics model fails past this point.")
				self.recordData([controlInput[0], controlInput[1], controlInput[2], controlInput[3]])
    
			# Full VTOL Demo
			# elif self.mode == 4:
				
			# 	dz = self.x[8]
			# 	pd = self.x[2]
				

			# 	if self.STATE == TAKEOFF:
			# 		try:
			# 			if pd <= -100:
			# 				prev_time = self.time
			# 				self.STATE = HOVER
			# 			controlInput = np.array([3000, -3000, 3000, -3000])
			# 			self.x = self.underlyingModel.update(self.x, controlInput)
			# 		except:
			# 			self.crashFlag = True
			# 			print("Takeoff Fail")
			# 			print("Quadrotor is pitched over. Dynamics model fails past this point.")
			# 		self.recordData([controlInput[0], controlInput[1], controlInput[2], controlInput[3]])
			# 	elif self.STATE == HOVER:  # hover mode
			# 		try:
			# 			controlInput = np.array([1000, -1000, 1000, -1000])
			# 			if dz > 0:
			# 				controlInput = np.array([2500, -2500, 2500, -2500])
			# 			self.x = self.underlyingModel.update(self.x, controlInput)
			# 			if (self.time - prev_time) > 3:
			# 				self.STATE = LAND
			# 		except:
			# 			self.crashFlag = True
			# 			print("hover Fail")
			# 			print("Quadrotor is pitched over. Dynamics model fails past this point.")
			# 		self.recordData([controlInput[0], controlInput[1], controlInput[2], controlInput[3]])
			# 	elif self.STATE == LAND:
			# 		try:
			# 			posz = self.x[2]
			# 			controlInput = np.array([1000, -1000, 1000, -1000])
			# 			if posz < -0.046:
			# 				controlInput = np.array([750, -750, 750, -750])
			# 			elif posz >= -0.046:
			# 				controlInput = np.array([0, 0, 0, 0])
			# 			self.x = self.underlyingModel.update(self.x, controlInput)
			# 		except:
			# 			self.crashFlag = True
			# 			print("Land fail")
			# 			print("Quadrotor is pitched over. Dynamics model fails past this point.")
			# 		self.recordData([controlInput[0], controlInput[1], controlInput[2], controlInput[3]])
		else:
			return
		return

	def reset(self):
		self.time = 0
		self.crashFlag = False
		
		if self.mode == 0 or self.mode == 2 or self.mode == 3: # free mode or hover mode
			self.x = np.array([0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		elif self.mode == 1 or self.mode == 4: # take off mode
			self.x = np.array([0, 0, -0.046, 0, 0, 0, 0, 0, 0, 0, 0, 0])
		
		self.underlyingModel = QuadrotorModel.QuadrotorModel(quad.quad, x0=self.x, dT = self.dT)
		self.takenData.clear()