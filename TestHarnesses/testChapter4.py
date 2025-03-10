"""
Author: Eric Vetha (evetha@ucsc.edu)

This file contains a test harness for the functions in VehicleAerodynamicsModel.py
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
import ece163.Modeling.VehicleAerodynamicsModel as VAM

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

# =============================================================================
# Tests Start Here
# =============================================================================

print("Beginning testing of VehicleAerodynamicsModel.py")

# =============================================================================

print()
print("Testing gravityForces()...")

cur_test = "Gravity Forces Test 1"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0)
								
result = testVAM.gravityForces(testState)
resultVec = [[result.Fx],[result.Fy],[result.Fz],[result.Mx],[result.My],[result.Mz]]
expected = Inputs.forcesMoments(Fx=0.0, Fy=0.0, Fz=107.91000000000001, Mx=0.0, My=0.0, Mz=0.0)
expectedVec = [[expected.Fx],[expected.Fy],[expected.Fz],[expected.Mx],[expected.My],[expected.Mz]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")
    
cur_test = "Gravity Forces Test 2"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=1, pe=1, pd=1, u=1, v=1, w=1, yaw=1, pitch=1, roll=1, p=1, q=1, r=1)

result = testVAM.gravityForces(testState)
resultVec = [[result.Fx],[result.Fy],[result.Fz],[result.Mx],[result.My],[result.Mz]]
expected = Inputs.forcesMoments(Fx=-90.80313397062012, Fy=49.061142664379666, Fz=31.501797434098943, Mx=0.0, My=0.0, Mz=0.0)
expectedVec = [[expected.Fx],[expected.Fy],[expected.Fz],[expected.Mx],[expected.My],[expected.Mz]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")
	
cur_test = "Gravity Forces Test 3"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=2, pe=2, pd=2, u=2, v=2, w=2, yaw=2, pitch=2, roll=2, p=2, q=2, r=2)

result = testVAM.gravityForces(testState)
resultVec = [[result.Fx],[result.Fy],[result.Fz],[result.Mx],[result.My],[result.Mz]]
expected = Inputs.forcesMoments(Fx=-98.12228532875932, Fy=-40.83327863433927, Fz=18.68765843630382, Mx=0.0, My=0.0, Mz=0.0)
expectedVec = [[expected.Fx],[expected.Fy],[expected.Fz],[expected.Mx],[expected.My],[expected.Mz]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

# =============================================================================

print()
print("Testing CalculateCoeff_alpha()...")

cur_test = "Calculate Coeff alpha Test 1"

testVAM = VAM.VehicleAerodynamicsModel()
testAlpha = 0

C_L, C_D, C_M = testVAM.CalculateCoeff_alpha(testAlpha)
resultVec = [[C_L],[C_D],[C_M]]
expectedVec = [[0.2299999999730887], [0.06122729464970145], [0.0135]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

cur_test = "Calculate Coeff alpha Test 2"

testVAM = VAM.VehicleAerodynamicsModel()
testAlpha = 2

C_L, C_D, C_M = testVAM.CalculateCoeff_alpha(testAlpha)
resultVec = [[C_L],[C_D],[C_M]]
expectedVec = [[-0.7568024953079283], [1.653643620863612], [-5.466500000000001]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

cur_test = "Calculate Coeff alpha Test 3"

testVAM = VAM.VehicleAerodynamicsModel()
testAlpha = 1

C_L, C_D, C_M = testVAM.CalculateCoeff_alpha(testAlpha)
resultVec = [[C_L],[C_D],[C_M]]
expectedVec = [[0.9092974268419374], [1.41614683654528], [-2.7265]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

# =============================================================================

print()
print("Testing aeroForces()...")

cur_test = "Aero Forces Test 1"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0)

result = testVAM.aeroForces(testState)
resultVec = [[result.Fx],[result.Fy],[result.Fz],[result.Mx],[result.My],[result.Mz]]
expected = Inputs.forcesMoments(Fx=0.0, Fy=0.0, Fz=0.0, Mx=0.0, My=0.0, Mz=0.0)
expectedVec = [[expected.Fx],[expected.Fy],[expected.Fz],[expected.Mx],[expected.My],[expected.Mz]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

cur_test = "Aero Forces Test 2"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=1, pe=1, pd=1, u=1, v=1, w=1, yaw=1, pitch=1, roll=1, p=1, q=1, r=1)

result = testVAM.aeroForces(testState)
resultVec = [[result.Fx],[result.Fy],[result.Fz],[result.Mx],[result.My],[result.Mz]]
expected = Inputs.forcesMoments(Fx=0.3224929693700297, Fy=-0.6310757798441824, Fz=-1.8021350237920357, Mx=-0.900820438947442, My=-0.8413294330659301, Mz=0.07027663000122117)
expectedVec = [[expected.Fx],[expected.Fy],[expected.Fz],[expected.Mx],[expected.My],[expected.Mz]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

cur_test = "Aero Forces Test 3"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=2, pe=2, pd=2, u=2, v=2, w=2, yaw=2, pitch=2, roll=2, p=2, q=2, r=2)

result = testVAM.aeroForces(testState)
resultVec = [[result.Fx],[result.Fy],[result.Fz],[result.Mx],[result.My],[result.Mz]]
expected = Inputs.forcesMoments(Fx=1.2899718774801188, Fy=-2.5243031193767296, Fz=-7.208540095168143, Mx=-3.603281755789768, My=-3.3653177322637204, Mz=0.2811065200048847)
expectedVec = [[expected.Fx],[expected.Fy],[expected.Fz],[expected.Mx],[expected.My],[expected.Mz]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

# =============================================================================

print()
print("Testing CalculatePropForces()...")

cur_test = "Calculate Prop Forces Test 1"

testVAM = VAM.VehicleAerodynamicsModel()
testVA = 0
testThrottle = 0

Fprop, Mprop = testVAM.CalculatePropForces(testVA, testThrottle)
resultVec = [[Fprop],[Mprop], [0.0]]
expectedVec = [[0.00018320594739300043], [-5.201975946047016e-06], [0.0]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

cur_test = "Calculate Prop Forces Test 2"

testVAM = VAM.VehicleAerodynamicsModel()
testVA = 2
testThrottle = 2

Fprop, Mprop = testVAM.CalculatePropForces(testVA, testThrottle)
resultVec = [[Fprop],[Mprop], [0.0]]
expectedVec = [[312.9776691343659], [-9.162169682173458], [0.0]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

cur_test = "Calculate Prop Forces Test 3"

testVAM = VAM.VehicleAerodynamicsModel()
testVA = 1
testThrottle = 1

Fprop, Mprop = testVAM.CalculatePropForces(testVA, testThrottle)
resultVec = [[Fprop],[Mprop], [0.0]]
expectedVec = [[83.39978539318385], [-2.4391985428760563], [0.0]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

# =============================================================================

print()
print("Testing controlForces()...")

cur_test = "Control Forces Test 1"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0)
testControls = Inputs.controlInputs()

result = testVAM.controlForces(testState, testControls)
resultVec = [[result.Fx],[result.Fy],[result.Fz],[result.Mx],[result.My],[result.Mz]]
expected = Inputs.forcesMoments(Fx=21.817680754436033, Fy=0.0, Fz=0.0, Mx=-0.6194943564776727, My=-0.0, Mz=-0.0)
expectedVec = [[expected.Fx],[expected.Fy],[expected.Fz],[expected.Mx],[expected.My],[expected.Mz]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

cur_test = "Control Forces Test 2"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=1, pe=1, pd=1, u=1, v=1, w=1, yaw=1, pitch=1, roll=1, p=1, q=1, r=1)
testControls = Inputs.controlInputs()

result = testVAM.controlForces(testState, testControls)
resultVec = [[result.Fx],[result.Fy],[result.Fz],[result.Mx],[result.My],[result.Mz]]
expected = Inputs.forcesMoments(Fx=20.761150339035364, Fy=0.0, Fz=0.0, Mx=-0.648316466476403, My=-0.0, Mz=-0.0)
expectedVec = [[expected.Fx],[expected.Fy],[expected.Fz],[expected.Mx],[expected.My],[expected.Mz]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

cur_test = "Control Forces Test 3"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=2, pe=2, pd=2, u=2, v=2, w=2, yaw=2, pitch=2, roll=2, p=2, q=2, r=2)
testControls = Inputs.controlInputs()

result = testVAM.controlForces(testState, testControls)
resultVec = [[result.Fx],[result.Fy],[result.Fz],[result.Mx],[result.My],[result.Mz]]
expected = Inputs.forcesMoments(Fx=19.513963664384256, Fy=0.0, Fz=0.0, Mx=-0.6610940195575558, My=-0.0, Mz=-0.0)
expectedVec = [[expected.Fx],[expected.Fy],[expected.Fz],[expected.Mx],[expected.My],[expected.Mz]]

if compareVectors(resultVec, expectedVec):
    passed.append(cur_test)
    print("    Passed!")
else:
    failed.append(cur_test)
    print("    Failed")

# =============================================================================

print()
print("Testing CalculateAirspeed()...")

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0)
testControls = Inputs.controlInputs()
testWind = States.windState()

result = testVAM.CalculateAirspeed(testState, testWind)
resultVec = [[result[1]], [result[0]], [result[2]]]
expected = [[0.0], [0.0], [0.0]]

if compareVectors(resultVec, expected):
    passed.append("Calculate Airspeed Test 1")
    print("    Passed!")
else:
    failed.append("Calculate Airspeed Test 1")
    print("    Failed")

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=1, pe=1, pd=1, u=1, v=1, w=1, yaw=1, pitch=1, roll=1, p=1, q=1, r=1)
testControls = Inputs.controlInputs()
testWind = States.windState()

result = testVAM.CalculateAirspeed(testState, testWind)
resultVec = [[result[1]], [result[0]], [result[2]]]
expected = [[0.7853981633974483], [1.7320508075688772], [0.6154797086703874]]

if compareVectors(resultVec, expected):
    passed.append("Calculate Airspeed Test 2")
    print("    Passed!")
else:
    failed.append("Calculate Airspeed Test 2")
    print("    Failed")

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState(pn=2, pe=2, pd=2, u=2, v=2, w=2, yaw=2, pitch=2, roll=2, p=2, q=2, r=2)
testControls = Inputs.controlInputs()
testWind = States.windState()

result = testVAM.CalculateAirspeed(testState, testWind)
resultVec = [[result[1]], [result[0]], [result[2]]]
expected = [[0.7853981633974483], [3.4641016151377544], [0.6154797086703874]]

if compareVectors(resultVec, expected):
    passed.append("Calculate Airspeed Test 3")
    print("    Passed!")
else:
    failed.append("Calculate Airspeed Test 3")
    print("    Failed")

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]
else:
    print("All tests passed!")