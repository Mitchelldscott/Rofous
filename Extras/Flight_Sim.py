############################################################################################################
#	variableActual -> in the inertial frame
#	

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


MASS = 4
Fg = -9.81


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


def get_netForce(pose, omegas, mass, velocity, k=0.005, b=-0.003):
	fNet = {'x':0, 'y':0, 'z': mass * Fg, 'phi':0, 'theta':0, 'psi':0}
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
	drag = R(pose, flopper=[b * velocity['x'], b * velocity['y'], b * velocity['z']])
	fNet['x'] += thrustActual['x'] + drag['x']
	fNet['y'] += thrustActual['y'] + drag['y']
	fNet['z'] += thrustActual['z'] + drag['z']
	return fNet


def step(target, pose, data):
	xComp = (data['gains'][0] * (target[0] - pose['x'])) - (data['gains'][1] * data['Vx'][-1]) - (data['gains'][2] * data['Fx'][-1])
	yComp = (data['gains'][0] * (target[1] - pose['y'])) - (data['gains'][1] * data['Vy'][-1]) - (data['gains'][2] * data['Fy'][-1])
	zComp = (data['gains'][0] * (target[2] - pose['z'])) - (data['gains'][1] * data['Vz'][-1]) - (data['gains'][2] * data['Fz'][-1])
	phiComp = (data['gains'][0] * (target[3] - pose['phi'])) - (data['gains'][1] * data['Vphi'][-1]) - (data['gains'][2] * data['Fphi'][-1])
	thetaComp = (data['gains'][0] * (target[4] - pose['theta'])) - (data['gains'][1] * data['Vtheta'][-1]) - (data['gains'][2] * data['Ftheta'][-1])
	psiComp = (data['gains'][0] * (target[5] - pose['psi'])) - (data['gains'][1] * data['Vpsi'][-1]) - (data['gains'][2] * data['Fpsi'][-1])
	data['omegas'][0] += -xComp + yComp + zComp - phiComp + thetaComp - psiComp
	data['omegas'][1] += -xComp - yComp + zComp + phiComp + thetaComp + psiComp
	data['omegas'][2] += xComp - yComp + zComp + phiComp - thetaComp - psiComp
	data['omegas'][3] += xComp + yComp + zComp - phiComp - thetaComp + psiComp
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


def draw(frame, axes, poseActual, netForce, target, data):
	data['t'] += 1
	velocity = {'x': netForce['x'] * data['CT'], 'y': netForce['y'] * data['CT'], 'z': netForce['z'] * data['CT'],
		'phi': netForce['phi'] * data['CT'], 'theta': netForce['theta'], 'psi': netForce['psi']}
	netForce = get_netForce(poseActual, data['omegas'], data['mass'], velocity)
	poseActual = update(poseActual, netForce, data['CT'])
	data = step(target, poseActual, data)
	t_span = np.linspace(0,data['t']-1,data['t'])
	data['Vx'] = np.concatenate([data['Vx'],[velocity['x']]]).reshape(t_span.shape)
	data['Vy'] = np.concatenate([data['Vy'],[velocity['y']]]).reshape(t_span.shape)
	data['Vz'] = np.concatenate([data['Vz'],[velocity['z']]]).reshape(t_span.shape)
	data['Vphi'] = np.concatenate([data['Vphi'],[velocity['phi']]]).reshape(t_span.shape)
	data['Vtheta'] = np.concatenate([data['Vtheta'],[velocity['theta']]]).reshape(t_span.shape)
	data['Vpsi'] = np.concatenate([data['Vpsi'],[velocity['psi']]]).reshape(t_span.shape)
	data['Fx'] = np.concatenate([data['Fx'],[netForce['x']]]).reshape(t_span.shape)
	data['Fy'] = np.concatenate([data['Fy'],[netForce['y']]]).reshape(t_span.shape)
	data['Fz'] = np.concatenate([data['Fz'],[netForce['z']]]).reshape(t_span.shape)
	data['Fphi'] = np.concatenate([data['Fphi'],[netForce['phi']]]).reshape(t_span.shape)
	data['Ftheta'] = np.concatenate([data['Ftheta'],[netForce['theta']]]).reshape(t_span.shape)
	data['Fpsi'] = np.concatenate([data['Fpsi'],[netForce['psi']]]).reshape(t_span.shape)
	axes[1].clear()
	h1, = axes[1].plot(t_span, data['Vx'], color='r', label='X-Velocity')
	h2, = axes[1].plot(t_span, data['Vy'], color='b', label='Y-Velocity')
	h3, = axes[1].plot(t_span, data['Vz'], color='g', label='Z-Velocity')
	axes[1].set_title('Velocities over time')
	axes[1].legend()
	axes[2].clear()
	axes[2].plot(t_span, data['Fx'], color='r', label='X-Accel')
	axes[2].plot(t_span, data['Fy'], color='b', label='Y-Accel')
	axes[2].plot(t_span, data['Fz'], color='g', label='Z-Accel')
	axes[2].set_title('Accelerations over time')
	axes[2].legend()
	axes[3].bar([1,3,5,7], data['omegas'], color='g', tick_label=['Front Left', 'Front right','Rear Right', 'Rear Left'])
	axes[0].clear()
	renderDevice(axes[0], poseActual, netForce)


def main():
	cycleTime = .5
	omegas = [0, 0, 0, 0]
	t = 1
	data = {'t':t, 
			'CT':cycleTime, 
			'omegas':omegas, 
			'mass':MASS,
			'Vx':np.array([0]), 'Vy':np.array([0]), 'Vz':np.array([0]), 'Vphi':np.array([0]), 'Vtheta':np.array([0]), 'Vpsi':np.array([0]), 
			'Fx':np.array([0]), 'Fy':np.array([0]), 'Fz':np.array([MASS * Fg]), 'Fphi':np.array([0]), 'Ftheta':np.array([0]), 'Fpsi':np.array([0]),
			'gains':[.1, 1, 1.5]}
	poseActual = {'x':0, 'y':0, 'z':0, 'phi':0, 'theta':0, 'psi':0}
	netForce = {'x':0, 'y':0, 'z':MASS * Fg, 'phi':0, 'theta':0, 'psi':0}
	target = [0,0,10,0,0,0]#list(map(float, input('Enter Target Pose(i.e. 0 0 0 0 0 0):\n\t').split()))
	while len(target) != 6:
		print('Usage Error: target pose formatting')
		target = list(map(int, input('Enter Target Pose(i.e. 0 0 0 0 0 0):\n\t').split()))
	fig = plt.figure(figsize=(10,6))
	ax0 = fig.add_subplot(221, projection='3d')
	ax1 = fig.add_subplot(222)
	ax2 = fig.add_subplot(223)
	ax3 = fig.add_subplot(224)
	ani = FuncAnimation(fig, draw, 200, fargs=([ax0, ax1, ax2, ax3], poseActual, netForce, target, data), interval=10)
	plt.show()
	
		


if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		print('User Kill: quiting ...')