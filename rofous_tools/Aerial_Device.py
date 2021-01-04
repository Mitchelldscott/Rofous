"""
	Aerial device kinematics class
	Author : Mitchell Scott
		- misc4432@colorado.edu
	Project : Rofous
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from Transformer import Transformer
from mpl_toolkits.mplot3d import Axes3D

# Gravitational constant
G = -9.81

""" Mark I"""
class Aerial_Device:
	def __init__(self, motor_positions, motor_orientations, motor_spin, mass, max_thrust, drag_coef, cycle_time=0.05):
		assert motor_positions.shape == motor_orientations.shape, f'Motor configuration is contradicting: {motor_positions.shape} != {motor_orientations.shape}' 
		assert len(motor_spin) == len(motor_positions), f'Motor spin and Motor positions do not algin: {len(motor_spin)} != {len(motor_positions)}'
		assert max_thrust > 0, f'Illegal Max Thrust Speed: {max_thrust} !> 0' # all motors must be identical for now
		assert mass > 0, f'Illegal Mass: {mass} !> 0'
		
		self.path = [[0], [0], [0]]
		self.mass = mass
		self.t = cycle_time
		self.autobots = None
		self.drag = drag_coef
		self.p = max_thrust / 10 # assuming ideal condiditons
		self.motor_spin = motor_spin
		self.max_thrust = max_thrust
		self.refActual = np.zeros(3)
		self.linearVel = np.zeros(3)
		self.poseActual = np.zeros(3)
		self.referenceVel = np.zeros(3)
		self.init_m_pose = motor_positions # motor positions relative to the body
		self.init_m_ref = motor_orientations # motor orientations relative to the body
		self.nMotors =  motor_positions.shape[0]
		self.throttleActual = np.zeros(self.nMotors)
		
		self.build_transformers()
		
	def wrap_angle(self, radians):
		return (radians + np.pi) % (2 * np.pi) - np.pi
		
	def build_transformers(self):
		# only needs to happen once if motors dont move w.r.t. the body frame
		# body frame is the devices reference to the world (maybe becomes adaptive someday)
		self.autobots = [Transformer(self.refActual[0], self.refActual[1], self.refActual[2], 'Body', translation=self.poseActual)]
		for i in range(self.nMotors):
			self.autobots.append(Transformer(self.init_m_ref[i,0], self.init_m_ref[i,1], self.init_m_ref[i,2], f'Motor{i}', translation=self.init_m_pose[i], parent=self.autobots[0]))
			
	def get_net_force(self):
		lin_thrust =  np.zeros(3)
		torque = np.zeros(3)
		# find the linear thrust and torque of each motor
		for i, throttle in enumerate(self.throttleActual):
			thrust = throttle**2 * self.p
			d = np.linalg.norm(self.init_m_pose[i])
			lin_thrust += self.autobots[i+1].transform(np.array([0.0, 0.0, thrust]), inverse=True) + (self.drag * self.linearVel)
			torque += self.autobots[i+1].transform(np.array([-thrust * d, 0.0, thrust * self.motor_spin[i]]), inverse=True) + (self.drag * self.referenceVel)
		return lin_thrust + (0.0, 0.0, G * self.mass), torque
	
	def update_odometry(self, t=-1):
		if t == -1:
			t = self.t
			
		lin_thrust, torque = self.get_net_force()
		lin_accel = lin_thrust / self.mass
		ang_accel = torque / self.mass
		self.poseActual += (self.linearVel * t) + (lin_accel * (t**2) / 2)
		self.refActual += (self.referenceVel * t) + (ang_accel * (t**2) / 2)
		self.linearVel += lin_accel * t
		self.referenceVel += ang_accel * t
		for i in range(len(self.refActual)):
			self.refActual[i] = self.wrap_angle(self.refActual[i])
			self.referenceVel[i] = self.wrap_angle(self.referenceVel[i])
		self.autobots[0].build_transform(self.refActual[0], self.refActual[1], self.refActual[2], self.poseActual)
		self.path[0].append(self.poseActual[0])
		self.path[1].append(self.poseActual[1])
		self.path[2].append(self.poseActual[2])
	
	def set_throttle(self, throttle):
		# throttles must be between 0 and 100
		assert len(throttle) == self.nMotors, f'Cannot set throttle: shape mismatch {len(throttle)} != {self.nMotors}'
		for i,throt in enumerate(throttle):
			self.throttleActual[i] = min(100.0, max(0.0, throt))

""" This class is used for rendering and debugging the device"""
class Renderer():
	def __init__(self, device):
		self.device = device

	def render_device(self, ax, colors=['r', 'b'], verbose=0):
		""" plot the device and net-force indicator on a pre-made 3D axis object
			Does not call show() that must be done in main or in animate"""

		# plot motors as points based on motor configuration
		# plot lines from (0,0)_i -> motor_k_i
		for i,motor in enumerate(self.device.init_m_pose):
			motor_pose = self.device.autobots[i+1].transform(np.array([0.0,0.0,0.0]), inverse=True)
			ax.scatter(motor_pose[0], motor_pose[1], motor_pose[2], color=colors[i % 2], marker='o', s=20)
			body_links = np.array(list(zip(self.device.poseActual, motor_pose)))
			ax.plot(body_links[0], body_links[1], body_links[2], color='black', lw=3)

		# plot the pose history
		ax.plot(self.device.path[0][-10:], self.device.path[1][-10:], self.device.path[2][-10:], color='c', lw=2)

		ax.set_xlim((self.device.poseActual[0] - 10, self.device.poseActual[0] + 10))
		ax.set_ylim((self.device.poseActual[1] - 10, self.device.poseActual[1] + 10))
		ax.set_zlim((self.device.poseActual[2] - 10, self.device.poseActual[2] + 10))
		ax.set_xlabel('X')
		ax.set_ylabel('Y')
		# print status
		if verbose > 0:
			print(f'New Pose:\n\tX : {self.device.poseActual[0]} \tY: {self.device.poseActual[1]} \tZ: {self.device.poseActual[2]} \tPhi: {self.device.poseActual[3]} \tTheta: {self.device.poseActual[4]} \tPsi: {self.device.poseActual[5]}')

	def render(self, colors=['r','b','g','c','m','y'], v=0):
		""" Current render method pretty weak
		It would be nice to not use matplotlib"""
		fig = plt.figure()
		axes = fig.add_subplot(221, projection='3d')

		self.render_device(axes, colors=colors[:2], verbose=v)

		plt.show()