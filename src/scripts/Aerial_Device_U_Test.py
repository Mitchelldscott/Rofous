"""
	Aerial device kinematics class test script
	Author : Mitchell Scott
		- misc4432@colorado.edu
	Project : Rofous
"""

import os
import sys
import time
import json
import numpy as np
import Aerial_Device
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

G = -9.81


def magnitude_of_(vec):
	acc = 0
	for i in vec:
		acc += i**2
	return np.sqrt(acc)

class Device_Doctor():
	def __init__(self, protocol={}, e_log='Device-Doctor-Error-Log.txt', test_data_path='AD_Tests.json'):
		self._log = {}
		self.log_file = e_log
		self.protocol = protocol
		self.name_tag = '[Device-Doctor]'
		self.device = Aerial_Device.Aerial_Device()
		self.configs = self.load_test_data(test_data_path)
		self.tags = self.configs['Tags']
		for test in self.tags:
			self._log[test] = []

	def load_test_data(self, filename):
		curr_dir = os.getcwd()
		path = ''

		items =  os.listdir(curr_dir)
		if 'Buffs' in items:
			os.chdir('Buffs')
			curr_dir = os.getcwd()
			items = os.listdir(curr_dir)
		if 'configs' in items:
			path = os.path.join(curr_dir, 'configs', filename)
			print(f'{self.name_tag} Found configs:\n\t reading from: {path}\n')
		elif filename in items:
			path = os.path.join(curr_dir, filename)
			print(f'{self.name_tag} Found data file:\n\t reading from: {path}\n')

		assert not path == '', f'{self.name_tag} didn\'t find test configs data\n make sure you run this from the repository\'s root directory'
		
		with open(path, 'r') as f:
			data = json.load(f)

		return data


	def compare_results(self, result, solution, tag, i, labels=('X', 'Y', 'Z'), tolerance=1e-6):
		score = 0

		if not result.shape[0] == len(solution):
			self.log(tag, f'{i}: shape mismatch in compare_results\n\tresults: {result.shape}, solutions: {len(solution)}')
			score += 1
		if not len(result.shape) == 1:
			self.log(tag, f'{i}: shape error in compare_results: Only 1D numpy arrays allowed!')
			score += 1
		if len(labels) < len(result):
			self.log(tag, f'{i}: value labels are missing or of the wrong quantity')
			score += 1

		for j,value in enumerate(result):
			if not np.abs(value - solution[j]) < tolerance:
				self.log(tag, f'{i}:\n\tfound {value}, expected {solution[j]} for {labels[j]}')
				score += 1

		if score > 0:
			return 0

		return 1

	def dump_log(self):
		for key in self._log:
			if len(self._log[key]) < 1:
					continue
			print(f'\t{key}\n')
			for e in self._log[key]:
				print(e)
			print('\n')

	def archive_log(self):
		with open(self.log_file, '+a') as f:
			f.write(f'\n\n\t\t\t{datetime.now}\n\n')
			for key in self._log:
				if len(self._log[key]) < 1:
					continue
				f.write(f'\t{key}')
				for e in self._log[key]:
					f.write(e,'\n')

	def log(self, tag, message):
		self._log[tag].append(f'{tag}  {message}')

	def test_rotations(self, reset=0):

		for i,r in enumerate(self.configs['Rotations'][0]):
			result = self.device.R(np.array([1,2,3]), ref=r)[0]
			self.compare_results(result, self.configs['Rotations'][1][i], self.tags[0], i)

	def test_translations(self, reset=0):

		for i,r in enumerate(self.configs['Translations']):
			result = self.device.T(r, ref=[0,0,0])[:3]
			self.compare_results(result, self.configs['Translations'][i], self.tags[1], i)

	def test_transforms(self, reset=0):

		for i,r in enumerate(self.configs['Transforms']):
			pose = self.device.R(np.array([0,0,0,0,0,0]), ref=r[3:])
			result = self.device.T(pose, ref=r[:3])
			self.compare_results(result, self.configs['Transforms'][i], self.tags[2], i, labels=('X', 'Y', 'Z', 'Phi', 'Theta', 'Psi'))

	def test_pose_update(self, t, reset=0):
		self.device.id.configs['mass'] = 1

		for i,forces in enumerate(self.configs['Pose-Updates'][0]):
			self.device.reset()
			self.device.net_force = forces
			self.device.update_velocity_and_pose(t)
			result = self.device.poseActual

			self.compare_results(result, self.configs['Pose-Updates'][1][i], self.tags[3], i, labels=('X', 'Y', 'Z', 'Phi', 'Theta', 'Psi'))

	def test_force_update(self, t, reset=0):
		self.device.id.configs['mass'] = 1

		engine_muzzles = np.array([[1,1,1,1],[1,0,1,0],[1,1,0,0],[1,0,0,1]])

		X = []
		Y = []
		Z = []
		Phi = []
		Theta = []
		Psi = []

		for i,muzzle in enumerate(engine_muzzles):
			self.device.reset()
			self.device.set_throttles(muzzle * self.device.id.configs['max rpm'])
			self.device.update_net_force()
			X.append(self.device.net_force[0])
			Y.append(self.device.net_force[1])
			Z.append(self.device.net_force[2])
			Phi.append(self.device.net_force[3])
			Theta.append(self.device.net_force[4])
			Psi.append(self.device.net_force[5])

		labels = ['Full', 'Yaw', 'Pitch', 'Roll']
		fig, axes = plt.subplots(2,3)
		t = np.array([1,2,3,4])
		axes[0][0].set_title('X-Force')
		axes[0][0].bar(t, X, tick_label=labels)
		axes[0][1].set_title('Y-Force')
		axes[0][1].bar(t, Y, tick_label=labels)
		axes[0][2].set_title('Z-Force')
		axes[0][2].bar(t, Z, tick_label=labels)
		axes[1][0].set_title('Phi-Force')
		axes[1][0].bar(t, Phi, tick_label=labels)
		axes[1][1].set_title('Theta-Force')
		axes[1][1].bar(t, Theta, tick_label=labels)
		axes[1][2].set_title('Psi-Force')
		axes[1][2].bar(t, Psi, tick_label=labels)
		plt.tight_layout()
		plt.show()

	def launch_test(self, horizon, t=.05, reset=0, verbose=1, visualize=False):
		if reset:
			self.device.reset()

		omega = np.sqrt(np.abs(G * self.device.id.configs['mass'] / (4 * self.device.id.configs['speed2thrust']))) * 2
		self.device.set_throttles([omega, omega, omega, omega])

		for h in range(horizon):
			self.device.update_odometry(t=t)

			if self.device.poseActual[2] < 0:
				self.log(self.tags[5], f'{h}: Device Z-pose negative')
			if self.device.velocity[2] < 0:
				self.log(self.tags[5], f'{h}: Device Z-vel negative')


			if verbose > 1:
				print(f'{self.tags[5]} Data:')
				print(f'Pose:\n\t{device.poseActual}')
				print(f'Velocity:\n\t{device.velocity}')
				print(f'net Force:\n\t{device.net_force}')

		if visualize:
			self.device.render()

	def neutral_thrust_test(self, horizon, t=.05, reset=0, verbose=0, visualize=False):
		""" Helps to run launch before this """
		if reset:
			device.reset()

		omega = np.sqrt(np.abs(G * self.device.id.configs['mass'] / (4 * self.device.id.configs['speed2thrust'])))
		self.device.set_throttles([omega, omega, omega, omega])
		
		for h in range(horizon):
			self.device.update_odometry(t=t)

			if np.abs(self.device.net_force[2]) > 1e-6:
				self.log(self.tags[8], f'{h}: acceleration detected')

			if verbose > 0:
				print(f'{self.tags[6]} Data:')
				print(f'Pose:\n\t{self.device.poseActual}')
				print(f'Velocity:\n\t{self.device.velocity}')
				print(f'net Force:\n\t{self.device.net_force}')

			omega = np.sqrt(np.abs(G * self.device.id.configs['mass'] / (4 * self.device.id.configs['speed2thrust'])))
			self.device.set_throttles([omega, omega, omega, omega])

		if visualize:
			self.device.render()

	def throttle_test(self, omegas, horizon, watch_value=2, t=.05, reset=0, verbose=0, visualize=False, direction=1):
		if reset:
			self.device.reset()

		self.device.adjust_throttles(omegas)

		for h in range(horizon):
			self.device.update_odometry(t=t)

			if self.device.net_force[watch_value] < 1e-2:
				self.log(self.tags[8], f'{h}: value unchanged')

			if np.sum(self.device.net_force[:watch_value]) + np.sum(self.device.net_force[watch_value:]) > 1:
				self.log(self.tags[8], f'{h}: unkown extraneous force components: {self.device.net_force}')

		if visualize:
			self.device.render()

	def pid_control_test(self, target, horizon, reset=0, t=.05, verbose=0, visualize=False):
		if reset:
			self.device.reset()

		og_pose = self.device.poseActual

		net_force_z = 0

		for h in range(horizon):
			self.device.adjust_throttles(target=target)
			self.device.update_odometry(t=t)

			if verbose > 0:
				print(f'{self.tags[7]} Data:')
				print(f'Pose:\n\t{self.device.poseActual}')
				print(f'Velocity:\n\t{self.device.velocity}')
				print(f'net Force:\n\t{self.device.net_force}')

			if self.device.poseActual[2] < 0:
				self.log[self.tags[7]].append(f'{self.tags[7]} Device sinking!!')

			if magnitude_of_(target - self.device.poseActual) < magnitude_of_(target - og_pose):
				self.log[self.tags[7]].append(f'{self.tags[7]} Failed to converge!')

			self.log[self.tags[7]].append(f'{self.tags[7]} INFO[{h}]:\n\tThrottles: {self.device.throttle}\n\tForces: {self.device.net_force}\n')

		if visualize:
			self.device.render()

	def run(self, protocol=-1, reset=[], horizon=10, verbose=0, visualize=False, throttle=np.array([0,0,0,0])):
		t = .5
		if protocol == -1:
			protocol = self.protocol

		if len(reset) < len(protocol):
			reset = np.zeros(len(protocol))

		print(f'{self.name_tag} The lab recieved {len(protocol)} test order(s) and will begin testing immediately!\n\n')

		id = -1

		for frame, order in enumerate(protocol):
			test = order.lower()

			if test == 'rotations' or test == 'rotation' or test == 'rotate' or test == 'r':
				self.test_rotations(reset=reset[frame])
				id = 0

			if test == 'translations' or test == 'translation' or test == 'translate' or test == 't':
				self.test_translations(reset=reset[frame])
				id = 1

			if test == 'transformation' or test == 'transforms' or test == 'transform' or test == 'tf':
				self.test_transforms(reset=reset[frame])
				id = 2

			if test == 'pose-update' or test == 'odom-update' or test == 'pose' or test =='odom' or test == 'pu':
				self.test_pose_update(t, reset=reset[frame])
				id = 3

			if test == 'forces-update' or test == 'force-update' or test == 'forces' or test =='force' or test == 'fu':
				self.test_force_update(t, reset=reset[frame])
				id = 4

			if test == 'launch' or test == 'l':
				self.launch_test(horizon, t=t, reset=reset[frame], verbose=verbose, visualize=visualize)
				id = 5

			if test == 'neutral-thrust' or test == 'buoyancy' or test == 'nt' or test == 'hover':
				self.neutral_thrust_test(horizon, t=t, reset=reset[frame], verbose=verbose, visualize=visualize)
				id = 6

			if test == 'pid-control' or test =='convergence' or test =='pid':
				self.pid_control_test([5,0,0,0,0,0], horizon, t=t, reset=reset[frame], verbose=verbose, visualize=visualize)
				id = 7

			if test == 'throttle-control' or test == 'throttle' or test == 'th':
				self.throttle_test(throttle, horizon, reset=reset[frame], t=t, verbose=verbose, visualize=visualize)
				id = 8

			assert id >= 0, f'{self.name_tag} {frame}: {order} unknown, valid tests are:\n{self.tags}'

			if len(self._log[self.tags[id]]) == 0:
					self.log(self.tags[id], '\tFinished with zero errors!')
			id = -1

		self.dump_log()


def main():

	doc = Device_Doctor()
	doc.device.id.memory_length = 500

	#doc.run(protocol=['nt','y'], visualize=True, horizon=200)
	doc.launch_test(1)
	doc.neutral_thrust_test(50, visualize=True)
	doc.throttle_test(np.array([0,1,0,1]), 50, watch_value=0, visualize=True)
	#doc.neutral_thrust_test(50, visualize=True)
	#doc.throttle_test(np.array([2,2,-2,-2]), 5, watch_value=0)
	#doc.neutral_thrust_test(50, visualize=True)

	doc.dump_log()



if __name__=='__main__':
	try:
		main()
	except AssertionError as ae:
		print(ae)
	except KeyboardInterrupt:
		print('User Kill Recieved! : Exiting ...')