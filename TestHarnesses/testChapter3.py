"""
Author: Eric Vetha (evetha@ucsc.edu)
This file contains a test harness for the functions in VehicleDynamicsModel.py
"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Constants.VehiclePhysicalConstants as VPC

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
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


# #%% Derivative():
# print("Beginning testing of VDM.Derivative(), subtest of [pe,pn,pd]")

# cur_test = "Derivative test p_dot x dir"

# testVDM = VDM.VehicleDynamicsModel()
# testState = States.vehicleState()
# testFm = Inputs.forcesMoments()
# testState.pitch = 30*math.pi/180
# testState.R = Rotations.euler2DCM(0.0,testState.pitch,0.0)
# testState.u = 10
# testDot = testVDM.derivative(testState, testFm)

# print("With a velocity of u = 10 m/s, and pitch = 30deg:\n")
# resultPdot = [[testDot.pn],[testDot.pe],[testDot.pd]]
# expectedPdot = [[10*math.sqrt(3)/2],[0],[-10/2]]

# if compareVectors(resultPdot,expectedPdot):
#     passed.append(cur_test)
#     print("passed!")
# else:
#     failed.append(cur_test)
#     print("failed :(")

# =============================================================================
# Tests Start Here
# =============================================================================

print("Beginning testing of VDM.Derivative()")

cur_test = "Derivative test p_dot x dir"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testState.pitch = 30*math.pi/180
testState.R = Rotations.euler2DCM(0.0, testState.pitch, 0.0)
testState.u = 10
testDot = testVDM.derivative(testState, testFm)

resultPdot = [[testDot.pn], [testDot.pe], [testDot.pd]]
expectedPdot = [[10*math.sqrt(3)/2], [0], [-10/2]]

if compareVectors(resultPdot, expectedPdot):
    passed.append(cur_test)
    print("passed!")
else:
    failed.append(cur_test)
    print("failed :(")

cur_test = "Derivative test u_dot, v_dot, w_dot"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testFm.Fx, testFm.Fy, testFm.Fz = 10, 20, 30
testDot = testVDM.derivative(testState, testFm)

resultVdot = [[testDot.u], [testDot.v], [testDot.w]]
expectedVdot = [[10/VPC.mass], [20/VPC.mass], [30/VPC.mass]]

if compareVectors(resultVdot, expectedVdot):
    passed.append(cur_test)
    print("passed!")
else:
    failed.append(cur_test)
    print("failed :(")

cur_test = "Derivative test p_dot, q_dot, r_dot"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testState.p, testState.q, testState.r = 0.1, 0.2, 0.3
testDot = testVDM.derivative(testState, testFm)

resultAdot = [[testDot.p], [testDot.q], [testDot.r]]
expectedAdot = [[-0.04404983969891156], [0.03318942731277532], [-0.010653553553012477]]

if compareVectors(resultAdot, expectedAdot):
    passed.append(cur_test)
    print("passed!")
else:
    failed.append(cur_test)
    print("failed :(")

cur_test = "Derivative test p_dot, q_dot, r_dot with moments"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testFm.Mx, testFm.My, testFm.Mz = 1, 2, 3
testDot = testVDM.derivative(testState, testFm)

resultAdot = [[testDot.p], [testDot.q], [testDot.r]]
expectedAdot = [[1.4768496674866216], [1.7621145374449338], [1.8066018760462699]]

if compareVectors(resultAdot, expectedAdot):
    passed.append(cur_test)
    print("passed!")
else:
    failed.append(cur_test)
    print("failed :(")

#%% Rexp():
print("Beginning testing of VDM.Rexp()")

cur_test = "Rexp test with simple angular velocities"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testDot.p, testDot.q, testDot.r = 0.1, 0.2, 0.3
Rexp = testVDM.Rexp(0.01, testState, testDot)
expectedRexp = [[0.9999999998375, 1.5000024999125e-05, -9.999962499416667e-06], [-1.4999974999125e-05, 0.999999999875, 5.000074999708331e-06], [1.0000037499416665e-05, -4.9999249997083355e-06, 0.9999999999375]]

if compareVectors(Rexp, expectedRexp):
    passed.append(cur_test)
    print("passed!")
else:
    failed.append(cur_test)
    print("failed :(")

#%% IntegrateState():
print("Beginning testing of VDM.IntegrateState()")

cur_test = "IntegrateState test for position integration"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.pn, testState.pe, testState.pd = 1, 2, 3
testDot.pn, testDot.pe, testDot.pd = 0.1, 0.2, 0.3
newState = testVDM.IntegrateState(0.01, testState, testDot)

resultPos = [[newState.pn], [newState.pe], [newState.pd]]
expectedPos = [[1.001], [2.002], [3.003]]

if compareVectors(resultPos, expectedPos):
    passed.append(cur_test)
    print("passed!")
else:
    failed.append(cur_test)
    print("failed :(")

cur_test = "IntegrateState test for velocity integration"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.u, testState.v, testState.w = 1, 2, 3
testDot.u, testDot.v, testDot.w = 0.1, 0.2, 0.3
newState = testVDM.IntegrateState(0.01, testState, testDot)

resultVel = [[newState.u], [newState.v], [newState.w]]
expectedVel = [[1.001], [2.002], [3.003]]

if compareVectors(resultVel, expectedVel):
    passed.append(cur_test)
    print("passed!")
else:
    failed.append(cur_test)
    print("failed :(")

cur_test = "IntegrateState test for angular rates integration"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.p, testState.q, testState.r = 0.1, 0.2, 0.3
testDot.p, testDot.q, testDot.r = 0.01, 0.02, 0.03
newState = testVDM.IntegrateState(0.01, testState, testDot)

resultAngRates = [[newState.p], [newState.q], [newState.r]]
expectedAngRates = [[0.10010000000000001], [0.20020000000000002], [0.3003]]

if compareVectors(resultAngRates, expectedAngRates):
    passed.append(cur_test)
    print("passed!")
else:
    failed.append(cur_test)
    print("failed :(")

cur_test = "IntegrateState test for angular acceleration integration"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.yaw, testState.pitch, testState.roll = 0.1, 0.2, 0.3
testDot.yaw, testDot.pitch, testDot.roll = 0.01, 0.02, 0.03
newState = testVDM.IntegrateState(0.01, testState, testDot)

resultAngAccel = [[newState.yaw], [newState.pitch], [newState.roll]]
expectedAngAccel = [[0.0], [-0.0], [0.0]]

if compareVectors(resultAngAccel, expectedAngAccel):
    passed.append(cur_test)
    print("passed!")
else:
    failed.append(cur_test)
    print("failed :(")

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]