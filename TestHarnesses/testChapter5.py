"""
Author: Eric Vetha (evetha@ucsc.edu)

This file is a test harness for the module VehiclePerturbationModels.
"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Modeling.WindModel as WM
import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States

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


vTrim1 = VehicleTrim.VehicleTrim()
Vastar1 = 25.0
Gammastar1 = math.radians(6.0)
Kappastar1 = -1.0 / 150.0

check1 = vTrim1.computeTrim(Vastar1, Kappastar1, Gammastar1)
evaluateTest("Test 1: Optimization successful", check1)

if check1:
    tF1 = VPM.CreateTransferFunction(
        vTrim1.getTrimState(), 
        vTrim1.getTrimControls()
    )

vTrim2 = VehicleTrim.VehicleTrim()
Vastar2 = 26.0
Gammastar2 = math.radians(6.0)
Kappastar2 = -1.0 / 150.0

check2 = vTrim2.computeTrim(Vastar2, Kappastar2, Gammastar2)
evaluateTest("Test 2: Optimization successful", check2)

if check2:
    tF2 = VPM.CreateTransferFunction(
        vTrim2.getTrimState(), 
        vTrim2.getTrimControls()
    )

vTrim3 = VehicleTrim.VehicleTrim()
Vastar3 = 20.0
Gammastar3 = math.radians(4.0)
Kappastar3 = -1.0 / 100.0

check3 = vTrim3.computeTrim(Vastar3, Kappastar3, Gammastar3)
evaluateTest("Test 3: Optimization successful", check3)

if check3:
    tF3 = VPM.CreateTransferFunction(
        vTrim3.getTrimState(), 
        vTrim3.getTrimControls()
    )

vTrim4 = VehicleTrim.VehicleTrim()
Vastar4 = 21.0
Gammastar4 = math.radians(4.0)
Kappastar4 = -1.0 / 100.0

check4 = vTrim4.computeTrim(Vastar4, Kappastar4, Gammastar4)
evaluateTest("Test 4: Optimization successful", check4)

if check4:
    tF4 = VPM.CreateTransferFunction(
        vTrim4.getTrimState(), 
        vTrim4.getTrimControls()
    )

vTrim5 = VehicleTrim.VehicleTrim()
Vastar5 = 22.0
Gammastar5 = math.radians(4.0)
Kappastar5 = -1.0 / 100.0

check5 = vTrim5.computeTrim(Vastar5, Kappastar5, Gammastar5)
evaluateTest("Test 5: Optimization successful", check5)

if check5:
    tF5 = VPM.CreateTransferFunction(
        vTrim5.getTrimState(), 
        vTrim5.getTrimControls()
    )


#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]