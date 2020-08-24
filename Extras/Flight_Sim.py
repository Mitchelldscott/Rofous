############################################################################################################
#	variableActual -> in the inertial frame
#	

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


def R(pose, flopper=[0, 0, 0]):
	return {'x' : (flopper[0] * np.cos(pose['psi']) * np.cos(pose['theta'])) + (flopper[1] * (np.cos(pose['psi']) * np.sin(pose['phi']) * np.sin(pose['theta']) - (np.sin(pose['psi']) * np.cos(pose['phi'])))) + (flopper[2] * (np.cos(pose['psi']) * np.sin(pose['theta']) * np.cos(pose['phi']) + (np.sin(pose['psi']) * np.sin(pose['phi'])))) + pose['x'],
		'y' : (flopper[0] * np.sin(pose['psi']) * np.cos(pose['theta'])) + (flopper[1] * (np.sin(pose['psi']) * np.sin(pose['phi']) * np.sin(pose['theta']) + (np.cos(pose['psi']) * np.cos(pose['phi'])))) + (flopper[2] * (np.sin(pose['psi']) * np.sin(pose['theta']) * np.cos(pose['phi']) - (np.cos(pose['psi']) * np.sin(pose['phi'])))) + pose['y'], 
		'z' : (flopper[0] * -np.sin(pose['theta'])) + (flopper[1] * np.cos(pose['theta']) * np.sin(pose['phi'])) + (flopper[2] * np.cos(pose['theta']) * np.cos(pose['phi'])) + pose['z']}


def update(pose, deltas, cycleTime):
	pose['x'] += deltas['x'] * cycleTime**2
	pose['y'] += deltas['y'] * cycleTime**2
	pose['z'] += deltas['z'] * cycleTime**2
	if pose['z'] < 0:
		pose['z'] = 0.0
	pose['phi'] += np.radians(deltas['phi']) * cycleTime**2
	pose['theta'] += np.radians(deltas['theta']) * cycleTime**2
	pose['psi'] += np.radians(deltas['psi']) * cycleTime**2
	return pose


def get_netForce(pose, omegas, mass, velocity, k=0.005, b=0.003):
	fNet = {'x':0, 'y':0, 'z':-mass * 9.81, 'phi':0, 'theta':0, 'psi':0}
	thrusts = []
	for i,omega in enumerate(omegas):
		if i < len(omegas) / 4 or i >= 3 * len(omegas) / 4:
			fNet['phi'] += omega**2
		else:
			fNet['phi'] -= omega**2
		if i < len(omegas) / 2:
			fNet['theta'] += omega**2
		else:
			fNet['theta'] -= omega**2
		fNet['psi'] += omega**2 * (-1)**i
		thrusts.append(omega**2)
	thrustActual = R(pose, flopper=[0,0,np.sum(thrusts) * k])
	drag = R(pose, flopper=[-b * velocity['x'], -b * velocity['y'], -b * velocity['z']])
	fNet['x'] += thrustActual['x'] + drag['x']
	fNet['y'] += thrustActual['y'] + drag['y']
	fNet['z'] += thrustActual['z'] + drag['z']
	return fNet


def step(target, pose, data):
	xComp = data['gains'][0] * (target[0] - pose['x'])
	yComp = data['gains'][1] * (target[1] - pose['y'])
	zComp = data['gains'][2] * (target[2] - pose['z'])
	phiComp = data['gains'][3] * (target[3] - pose['phi'])
	thetaComp = data['gains'][4] * (target[4] - pose['theta'])
	psiComp = data['gains'][5] * (target[5] - pose['psi'])
	data['omegas'][0] += -xComp + yComp + zComp
	data['omegas'][1] += -xComp - yComp + zComp
	data['omegas'][2] += xComp - yComp + zComp
	data['omegas'][3] += xComp + yComp + zComp
	return data


def renderDevice(ax, pose, fNet, motors=[(1,1,0), (-1,1,0), (-1,-1,0), (1,-1,0)], colors=['r', 'b']):
	for i,motor in enumerate(motors):
		motorPose = R(pose, flopper=motor)
		ax.scatter(motorPose['x'], motorPose['y'], motorPose['z'], color=colors[i % 2], marker='o')
		ax.plot([pose['x'], motorPose['x']],
			[pose['y'], motorPose['y']],
			[pose['z'], motorPose['z']], color='black')
	ax.plot([pose['x'], fNet['x'] + pose['x']], [pose['y'], fNet['y'] + pose['y']], [pose['z'], fNet['z'] + pose['z']], color='y')
	print(f'New Pose {pose["x"]}\n\t{pose["y"]}\n\t{pose["z"]}\n\t{pose["phi"]}\n\t{pose["theta"]}\n\t{pose["psi"]}')
	print(f'Forces: {fNet["x"]}\n\t{fNet["y"]}\n\t{fNet["z"]}\n\t{fNet["phi"]}\n\t{fNet["theta"]}\n\t{fNet["psi"]}')
	ax.set_xlim3d(-5,5)
	ax.set_ylim3d(-5,5)
	ax.set_zlim3d(0,30)


def draw(frame, axes, poseActual, netForce, data):
	target = list(map(float, input('Enter Target Pose(i.e. 0 0 0 0 0 0):\n\t').split()))
	while len(target) != 6:
		print('Usage Error: target pose formatting')
		target = list(map(int, input('Enter Target Pose(i.e. 0 0 0 0 0 0):\n\t').split()))
	velocity = {'x': netForce['x'] * data['CT'], 'y': netForce['y'] * data['CT'], 'z': netForce['z'] * data['CT']}
	netForce = get_netForce(poseActual, data['omegas'], data['mass'], velocity)
	poseActual = update(poseActual, netForce, data['CT'])
	data = step(target, poseActual, data)
	data['t'].append(frame)
	axes[1].clear()
	axes[1].plot([0, velocity['x']],[0, velocity['y']], [0, velocity['z']], color='b')
	axes[1].plot([0, netForce['x']],[0, netForce['y']], [0, netForce['z']], color='r')
	axes[1].legend(['Velocity', 'Acceleration'])
	axes[2].bar([1,3,5,7], data['omegas'], color='g', tick_label=['Front Left', 'Front right','Rear Right', 'Rear Left'])
	axes[0].clear()
	renderDevice(axes[0], poseActual, netForce)


def main():
	cycleTime = .5
	omegas = [0, 0, 0, 0]
	t = [0]
	m = 4
	g = 9.81
	data = {'t':t, 
			'CT':cycleTime, 
			'omegas':omegas, 
			'mass':m, 
			'Fx': [0], 'Fy':[0], 'Fz':[-m * g], 'Fphi':[0], 'Ftheta':[0], 'Fpsi':[0],
			'gains':[1, 1, 1, 1, 1, 1]}
	poseActual = {'x':0, 'y':0, 'z':0, 'phi':0, 'theta':0, 'psi':0}
	netForce = {'x':0, 'y':0, 'z':-m * g, 'phi':0, 'theta':0, 'psi':0}
	fig = plt.figure(figsize=(10,6))
	ax0 = fig.add_subplot(221, projection='3d')
	ax1 = fig.add_subplot(222, projection='3d')
	ax2 = fig.add_subplot(223)
	ani = FuncAnimation(fig, draw, 200, fargs=([ax0, ax1, ax2], poseActual, netForce, data), interval=10)
	plt.show()
	
		


if __name__ == "__main__":
	main()