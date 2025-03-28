import subprocess
import sys
import os
import time

PackagesNeeded = ['requests', 'paramiko', 'PyQT5', 'pyserial', 'pyqtgraph', 'PyOpenGL', 'numpy', 'scipy', 'matplotlib','pygame']
print("This class requires python greater than 3.8")
if not (sys.version_info.major == 3 and sys.version_info.minor >= 8):
	print('Invalid version of python Detected.')
	sys.exit(-1)
print("This script installs the required packages for the class. If it fails the required ones are listed below")
print(' '.join(PackagesNeeded))

print('On Windows we assume pip3 is in the scripts directory')
print('On Posix we assume pip3 is on the path')
print('If your system is non standard install manually')
time.sleep(1)

if os.name == 'nt':
	pythonPath = sys.executable
	pythonDir = os.path.split(pythonPath)[0]
	# and then find the pip path
	pipPath = os.path.join(pythonDir, 'Scripts', 'pip3')
else:
	pipPath = 'pip3'
try:
	for package in PackagesNeeded:
		print('Installing '+package)
		subprocess.run([pipPath, 'install', '--upgrade', package])
except FileNotFoundError:
	print('pip3 not found, please install first')
