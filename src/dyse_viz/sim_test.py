#!/usr/bin/env python3

import time
import numpy as np
from simulation import *
from scipy import linalg as la

def DCM_tests():
	x = np.array([1, 0, 0])
	y = np.array([0, 1, 0])
	z = np.array([0, 0, 1])

	eye = DCM([0, 0, 0])
	assert np.sum(eye - np.eye(3)) == 0

	dcm = DCM([np.pi, 0, 0])
	z_rotated = z @ dcm
	y_rotated = y @ dcm
	x_rotated = x @ dcm
	assert sum(x_rotated - x) <= 1e-10
	assert sum(y_rotated + y) <= 1e-10
	assert sum(z_rotated + z) <= 1e-10

	dcm = DCM([0, np.pi, 0])
	z_rotated = z @ dcm
	y_rotated = y @ dcm
	x_rotated = x @ dcm
	assert sum(x_rotated + x) <= 1e-10
	assert sum(y_rotated - y) <= 1e-10
	assert sum(z_rotated + z) <= 1e-10

	dcm = DCM([0, 0, np.pi])
	z_rotated = z @ dcm
	y_rotated = y @ dcm
	x_rotated = x @ dcm
	assert sum(x_rotated + x) <= 1e-10
	assert sum(y_rotated + y) <= 1e-10
	assert sum(z_rotated - z) <= 1e-10

	print("DCM Tests passed")

def tilde_tests():
	rng = np.random.default_rng()
	for i in range(100):
		v1 = np.array([rng.standard_normal(), rng.standard_normal(), rng.standard_normal()])
		v2 = np.array([rng.standard_normal(), rng.standard_normal(), rng.standard_normal()])

		assert la.norm(np.cross(v1, v2)) - la.norm(tilde(v1) @ v2) <= 1e-10

	print("Tilde op tests passed")

if __name__ == '__main__':
	DCM_tests()
	tilde_tests()