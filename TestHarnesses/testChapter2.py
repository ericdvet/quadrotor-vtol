"""
Author: Eric Vetha (evetha@ucsc.edu)
This file contains a test harness for the functions in Rotations.py
"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-2)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-1], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))

failed = []
passed = []
def evaluateTest(test_name, boolean):
	"""evaluateTest prints the output of a test and adds it to one of two 
	global lists, passed and failed, which can be printed later"""
	if boolean:
		print(f"   passed {test_name}")
		passed.append(test_name)
	else:
		print(f"   failed {test_name}")
		failed.append(test_name)
	return boolean

print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm test 1"
R = Rotations.euler2DCM(1, 1, 1)
expected_dcm = [[0.2919, 0.4546, -0.8415], [-0.0721, 0.8877, 0.4546], [0.9537, -0.0721, 0.2919]]
if not evaluateTest(cur_test, all([isclose(R[i][j], expected_dcm[i][j]) for i in range(3) for j in range(3)])):
    print(f"Expected {expected_dcm}, got {R}")
    
cur_test = "Euler2dcm test 2"
R = Rotations.euler2DCM(0, 0, 0)
expected_dcm = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
if not evaluateTest(cur_test, all([isclose(R[i][j], expected_dcm[i][j]) for i in range(3) for j in range(3)])):
    print(f"Expected {expected_dcm}, got {R}")
    
cur_test = "Euler2dcm test 3"
R = Rotations.euler2DCM(-1, -1, -1)
expected_dcm = [[0.2919, -0.4546, 0.8415], [0.8372, -0.3039, -0.4546], [0.4624, 0.8372, 0.2919]]
if not evaluateTest(cur_test, all([isclose(R[i][j], expected_dcm[i][j]) for i in range(3) for j in range(3)])):
    print(f"Expected {expected_dcm}, got {R}")
    
print("Beginning testing of Rotations.dcm2Euler()")

cur_test = "dcm2Euler test 1"
dcm = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
expected = (0, 0, 0)
yaw, pitch, roll = Rotations.dcm2Euler(dcm)
actual = (yaw, pitch, roll)
if not evaluateTest(cur_test, isclose(actual[0], expected[0]) and isclose(actual[1], expected[1]) and isclose(actual[2], expected[2])):
    print(f"Expected {expected}, got {actual}")

cur_test = "dcm2Euler test 2"
dcm = Rotations.euler2DCM(90*math.pi/180, 0, 0)
expected = (90*math.pi/180, 0, 0)
yaw, pitch, roll = Rotations.dcm2Euler(dcm)
actual = (yaw, pitch, roll)
if not evaluateTest(cur_test, isclose(actual[0], expected[0]) and isclose(actual[1], expected[1]) and isclose(actual[2], expected[2])):
    print(f"Expected {expected}, got {actual}")

cur_test = "dcm2Euler test 3"
dcm = Rotations.euler2DCM(45*math.pi/180, 30*math.pi/180, 60*math.pi/180)
expected = (45*math.pi/180, 30*math.pi/180, 60*math.pi/180)
yaw, pitch, roll = Rotations.dcm2Euler(dcm)
actual = (yaw, pitch, roll)
if not evaluateTest(cur_test, isclose(actual[0], expected[0]) and isclose(actual[1], expected[1]) and isclose(actual[2], expected[2])):
    print(f"Expected {expected}, got {actual}")
    
print("Beginning testing of Rotations.ned2enu()")

cur_test = "ned2enu test 1"
ned_point = [[1, 2, 3]]
expected_point = [[2, 1, -3]]
actual_point = Rotations.ned2enu(ned_point)
actual_point = mm.transpose(actual_point)
expected_point = mm.transpose(expected_point)
if not evaluateTest(cur_test, compareVectors(expected_point, actual_point)):
    print(f"Expected {expected_point}, got {actual_point}")

cur_test = "ned2enu test 2"
ned_point = [[0, 0, 0]]
expected_point = [[0, 0, 0]]
actual_point = Rotations.ned2enu(ned_point)
actual_point = mm.transpose(actual_point)
expected_point = mm.transpose(expected_point)
if not evaluateTest(cur_test, compareVectors(expected_point, actual_point)):
    print(f"Expected {expected_point}, got {actual_point}")

cur_test = "ned2enu test 3"
ned_point = [[-1, -2, -3]]
expected_point = [[-2, -1, 3]]
actual_point = Rotations.ned2enu(ned_point)
actual_point = mm.transpose(actual_point)
expected_point = mm.transpose(expected_point)
if not evaluateTest(cur_test, compareVectors(expected_point, actual_point)):
    print(f"Expected {expected_point}, got {actual_point}")

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]