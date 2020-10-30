"""
	Aerial device simulation script
	Author : Mitchell Scott
		- misc4432@colorado.edu
	Project : Humming Bird
"""

import sys
import numpy as np
import Aerial_Device
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

G = -9.81
pose_ERR = 1e-2

def get_diff(pose1, pose2):
	acc1 = 0
	acc2 = 0
	for i,element in enumerate(pose1):
		acc1 += element**2
		acc2 += pose2[i]**2
	return np.sqrt(acc1) - np.sqrt(acc2)


def target_pose_sim(device, targets, reset=True):
	for target in targets:
		while get_diff(target, device.poseActual) > pose_ERR:




def main():
	horizon = 1000
	render_rate = 10
	device = Aerial_Device.Device()
	
	fig = plt.figure(figsize=(15,8))
	axes = [fig.add_subplot(221, projection='3d'),
			fig.add_subplot(222),
			fig.add_subplot(223),
			fig.add_subplot(224)]


if __name__=='__main__':
	try:
		main()
	except KeyboardInterrupt:
		print('[User Kill] Exiting ...')