from . import Simulate
from ..Modeling import VehicleAerodynamicsModel
from ..Containers.Inputs import controlInputs
from ..Constants import VehiclePhysicalConstants
from ..Quadrotor import QuadrotorPhysicalConstants as QPC
from ..Quadrotor import FlightControllerSystem as FSC
from ..Quadrotor import QuadrotorModel

class QuadrotorSimulate(Simulate.Simulate):
	def __init__(self):
		super().__init__()
		self.inputNames.extend(['Motor1', 'Motor2', 'Motor3', 'Motor4'])
		self.underlyingModel = QuadrotorModel.QuadrotorModel(QPC.quad)

		# self.variableList.append((self.underlyingModel.getForcesMoments, 'ForceMoments',
		# 							['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']))

		self.variableList.append((self.underlyingModel.getVehicleState, 'state',
									['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'Va', 'alpha', 'beta']))
		# self.dT = 1/50

		self.controlInput = FSC.FlightControllerSystem(self.underlyingModel.getVehicleState)

	def getVehicleState(self):
		return self.underlyingModel.getVehicleState()

	def takeStep(self, controlInput=None):
		self.time += QPC.dT
		if controlInput is None:
			controlInput = self.controlInput
		self.underlyingModel.update(self.underlyingModel.getVehicleState, controlInput)
		self.recordData([controlInput.Throttle, controlInput.Aileron, controlInput.Elevator, controlInput.Rudder])
		return

	def reset(self):
		self.time = 0
		self.underlyingModel.reset()
		self.takenData.clear()