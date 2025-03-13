"""
Holds the vehicle graphics, only operation on it is to return a set of points and meshes with the appropriate rotation/translation
currently just returns the modified points, does not update the base ones. Module uses its baseUnit variable to scale the model to an
arbitrary size for good rendering in the display window.
"""

from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

baseUnit = 1.0

class VehicleGeometry():
	def __init__(self):
		"""
		defines the vehicle in NED coordinates around the local body frame origin. Rotations and translations will be
		around the [0,0,0] point of this local frame. Has to be in NED or the rotation matrices will not work. Vehicle is
		scaled to match the wing span of the actual vehicle, this all the points are in meters.

		"vertices" is an [n x 3] matrix of xyz points for each vertex of the vehicle;
		"faces" is an [m x3] index matrix of which vertices connect to which face (only triangles allowed for faces);
		"colors" is an [m x 4] matrix where each row is a CMYK definition of that face color
		"""


		chassis_color = [1., 1., 0., 1] #yellow
		propeller_color = [1., 0., 0., 1] #red

		scalingUnit = VPC.b / 6.0  

		length_x = 5 * scalingUnit  
		width_x = 0.6 * scalingUnit  
		length_y = 5 * scalingUnit  
		width_y = 0.6 * scalingUnit  
		height = 0.3 * scalingUnit  

		prop_blade_length = 1 * scalingUnit  
		prop_blade_width = 0.1 * scalingUnit 
		prop_z_offset = 0.01 * scalingUnit  

		prop_x = length_x / 2  
		prop_y = 0  
		prop_z = height / 2 + prop_z_offset  

		propeller_positions = [
			(length_x / 2, 0),  # Right 
			(-length_x / 2, 0), # Left 
			(0, length_y / 2),  # Front 
			(0, -length_y / 2)  # Back 
		]

		self.vertices = [
			# x-axis rectangle
			[-length_x / 2, -width_x / 2, -height / 2],  
			[length_x / 2, -width_x / 2, -height / 2],  
			[length_x / 2, width_x / 2, -height / 2],  
			[-length_x / 2, width_x / 2, -height / 2],  
			[-length_x / 2, -width_x / 2, height / 2],  
			[length_x / 2, -width_x / 2, height / 2],  
			[length_x / 2, width_x / 2, height / 2],  
			[-length_x / 2, width_x / 2, height / 2],  

			# y-axis rectangle
			[-width_y / 2, -length_y / 2, -height / 2],  
			[width_y / 2, -length_y / 2, -height / 2],  
			[width_y / 2, length_y / 2, -height / 2],  
			[-width_y / 2, length_y / 2, -height / 2],  
			[-width_y / 2, -length_y / 2, height / 2],  
			[width_y / 2, -length_y / 2, height / 2],  
			[width_y / 2, length_y / 2, height / 2],  
			[-width_y / 2, length_y / 2, height / 2],  
		]

		# add propellers on all arms
		for prop_x, prop_y in propeller_positions:
			prop_z = height / 2 + prop_z_offset
			self.vertices.extend([
				# blade horizontal
				[prop_x - prop_blade_length / 2, prop_y - prop_blade_width / 2, prop_z],  
				[prop_x + prop_blade_length / 2, prop_y - prop_blade_width / 2, prop_z],  
				[prop_x + prop_blade_length / 2, prop_y + prop_blade_width / 2, prop_z],  
				[prop_x - prop_blade_length / 2, prop_y + prop_blade_width / 2, prop_z],  

				# blade vertical
				[prop_x - prop_blade_width / 2, prop_y - prop_blade_length / 2, prop_z],  
				[prop_x + prop_blade_width / 2, prop_y - prop_blade_length / 2, prop_z],  
				[prop_x + prop_blade_width / 2, prop_y + prop_blade_length / 2, prop_z],  
				[prop_x - prop_blade_width / 2, prop_y + prop_blade_length / 2, prop_z],  
			])

		self.faces = [
			[0, 1, 2], [0, 2, 3],  
			[4, 5, 6], [4, 6, 7],  
			[0, 1, 5], [0, 5, 4],  
			[2, 3, 7], [2, 7, 6],  
			[1, 2, 6], [1, 6, 5],  
			[3, 0, 4], [3, 4, 7],  

			[8, 9, 10], [8, 10, 11],  
			[12, 13, 14], [12, 14, 15],  
			[8, 9, 13], [8, 13, 12],  
			[10, 11, 15], [10, 15, 14],  
			[9, 10, 14], [9, 14, 13],  
			[11, 8, 12], [11, 12, 15],  
		]

		# faces for each prop
		for i in range(len(self.vertices) - 32, len(self.vertices), 8):  
			self.faces.extend([
				[i, i + 1, i + 2],  
				[i, i + 2, i + 3],  
				[i + 4, i + 5, i + 6],  
				[i + 4, i + 6, i + 7],  
			])

		# apply colors
		self.colors = [chassis_color] * (len(self.faces) - 16) + [propeller_color] * 16

		return

	def getNewPoints(self, x, y, z, yaw, pitch, roll):
		"""
		Function to get new ENU points of the vehicle in inertial space from Euler angles, NED displacements, and base
		drawing contained within the __init__ function. That is, points to be remapped are contained within self.vertices

		:param x: North Displacement (Pn) in [m]
		:param y: East Displacement (Pe) in [m]
		:param z: Down Displacement (Pd) in [m]
		:param yaw: rotation about inertial down [rad]
		:param pitch: rotation about intermediate y-axis [rad]
		:param roll: rotation about body x-axis [rad]
		:return: Points in inertial EAST-NORTH-UP frame (for plotting)
		"""
		newPoints = self.vertices
		R = Rotations.euler2DCM(yaw, pitch, roll)
		newPoints = MatrixMath.multiply(newPoints, R)
		newPoints = MatrixMath.offset(newPoints, x, y, z)
		newPoints = Rotations.ned2enu(newPoints)
		return newPoints
