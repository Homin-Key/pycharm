"""
ECE487 Spatial Math Exercise
"""
import numpy as np
from spatialmath.base import *
from spatialmath import *

# Using the ZYX Euler angles, find the rotation matrix Rzyx(psi,theta,phi)
# for psi = 45 deg, theta = 90 deg, and phi = 45 deg

# Use low-level spatial math functions to find Rzyx
R = rotz(45, 'deg')@roty(90, 'deg')@rotx(45, 'deg')
print(R)

# Find the Euler angles from Rzyx
rpy = tr2rpy(R, unit='deg')
print(rpy)

# Use high-level spatial math functions to find Rzyx
Rx = SO3.Rx(45, 'deg')
Ry = SO3.Ry(90, 'deg')
Rz = SO3.Rz(45, 'deg')
Rzyx = Rz * Ry * Rx   # We use * not @ for matrix multiplication for SO3 objects.
print(Rzyx)

# Find the Euler angles from Rzyx
float_formatter = "{:.2f}".format
np.set_printoptions(formatter={'float_kind':float_formatter})
print(Rzyx.rpy('deg'))  # Returnz
