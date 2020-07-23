import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def update(pose, deltas):
	pose['x'] += deltas[0]
	pose['y'] += deltas[1]
	pose['z'] += deltas[2]
	pose['roll'] += np.radians(deltas[3])
	pose['pitch'] += np.radians(deltas[4])
	pose['yaw'] += np.radians(deltas[5])


def spherical2cartesian(theta, phi):
	theta = np.radians(theta)
	phi = np.radians(phi)
	if theta == 'N/A':
		if phi == 'N/A':
			return (0, 0, 0)
		else:
			return (0, 0, 1)
	elif phi == 'N/A':
		return (np.cos(theta), np.sin(theta), 0)
	else:
		return (np.round(np.cos(theta)*np.sin(phi), 4), np.round(np.sin(theta)*np.sin(phi), 4), np.round(np.cos(phi), 4))


def translate(heading, deltas):
	adjustment = {'xpitch' : heading[0] - (deltas[0] + (deltas[4] / 2000)),
								'yroll' : heading[1] - (deltas[1] + (deltas[3] / 2000)),
								'z' : heading[2] - deltas[2],
								'yaw' : -deltas[5] / 2000}
	return adjustment


def render(pose, heading):
	fig = plt.figure(figsize=(8, 5))
	ax0 = fig.add_subplot(121, projection='3d')
	ax1 = fig.add_subplot(122, projection='3d')
	xVect = (np.round(np.cos(pose['yaw'])*np.cos(pose['pitch']), 4), np.round(-np.sin(pose['yaw']), 4), np.round(-np.sin(pose['pitch']), 4))
	yVect = (np.round(np.sin(pose['yaw']), 4), np.round(np.cos(pose['roll'])*np.cos(pose['yaw']), 4), np.round(np.sin(pose['roll']), 4))
	zVect = (np.round(np.sin(pose['pitch']), 4), np.round(np.sin(pose['roll']), 4), np.round(np.cos(pose['pitch'])*np.cos(pose['roll']), 4))
	ax0.plot([pose['x'], pose['x'] + xVect[0]], [pose['y'], pose['y'] + xVect[1]], [pose['z'], pose['z'] + xVect[2]], color='b')
	ax0.plot([pose['x'], pose['x'] + yVect[0]], [pose['y'], pose['y'] + yVect[1]], [pose['z'], pose['z'] +  yVect[2]], color='r')
	ax0.plot([pose['x'], pose['x'] + zVect[0]], [pose['y'], pose['y'] + zVect[1]], [pose['z'], pose['z'] +  zVect[2]], color='g')
	ax1.plot([pose['x'], pose['x'] + heading[0]], [pose['y'], pose['y'] + heading[1]], [pose['z'], pose['z'] + heading[2]], color='y')
	ax1.scatter([pose['x'] + heading[0]], [pose['y'] + heading[1]], [pose['z'] + heading[2]], color='purple', marker='^')
	ax0.scatter([7, 7, -7, -7], [7, -7, -7, 7], [0, 0, 0, 0], color='w')
	ax0.scatter([7, 7, -7, -7], [7, -7, -7, 7], [7, 7, 7, 7], color='w')
	ax1.scatter([2, 2, -2, -2], [2, -2, -2, 2], [-2, -2, -2, -2], color='w')
	ax1.scatter([2, 2, -2, -2], [2, -2, -2, 2], [2, 2, 2, 2], color='w')
	print(f'X unit vector: {xVect}')
	print(f'Y unit vector: {yVect}')
	print(f'Z unit vector: {zVect}')
	plt.show()


pose = {'x':0, 'y':0, 'z':0, 'roll':0, 'pitch':0, 'yaw':0}
heading = spherical2cartesian(0, 90)

deltas = [1, 0, 0, 0, 0, 0]
update(pose, deltas)
print(f'Adjustments {translate(heading, deltas)}')
print(f'Heading {heading}')
#render(pose, heading)


