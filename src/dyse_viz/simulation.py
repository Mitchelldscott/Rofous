#!/usr/bin/env python3

import tf
import time
import rospy
import tf2_ros
import numpy as np
import geometry_msgs.msg
from scipy import linalg as la

def DCM(q):
	Rx = np.array([[1,			  0,			 0], 
				   [0, np.cos(q[0]), -np.sin(q[0])],
				   [0, np.sin(q[0]), np.cos(q[0])]])

	Ry = np.array([[ np.cos(q[1]),  0, np.sin(q[1])],
				   [			0,  1,			  0],
				   [-np.sin(q[1]),  0, np.cos(q[1])]])

	Rz = np.array([[ np.cos(q[2]), -np.sin(q[2]), 0],
				   [ np.sin(q[2]),  np.cos(q[2]), 0],
				   [			0,			   0, 1]])

	return Rz @ Ry @ Rx;

def tilde(v):
	return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def euler_to_quaternion(yaw, pitch, roll):
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qx, qy, qz, qw]


class Rufous_Simulation():
	def __init__(self):
		#		States
		self.r = np.zeros(3)				# postion
		self.v = np.zeros(3)				# velocity
		self.q = np.zeros(3)				# attitude
		self.w = np.zeros(3)				# angular rate
		self.u = np.zeros(4)				# control output, input to the plant
		self.Tp = np.zeros(3)				# thrust from propellers
		self.Taup = np.zeros(3)				# torque from propellers


		#		Parameters
		self.g = 9.81						# gravitational constant
		self.Kt = 5							# thrust constant
		self.mass = 5						# quadcopter mass
		self.l = 0.1  						# wing length (center to prop rot axis)
		self.I = np.array([[1, 0, 0], 		# Inertia matrix
							[0, 1, 0], 
							[0, 0, 0.1]])

		self.hz = 10
		self.rate = rospy.Rate(self.hz)
		self.broadcaster = tf2_ros.StaticTransformBroadcaster()

	def thrust(self):
		Rnb = DCM(self.q)
		z_hat_b = Rnb.T @ np.array([0, 0, 1]).T
		self.Tp = self.Kt * np.sum(self.u**2) * z_hat_b
		return self.Tp

	def torque(self):
		self.Taup = self.Kt * self.l * np.array([[1, 0, -1, 0], [0, -1, 0, 1], [-1, 1, -1, 1]]) @ (self.u**2)
		return self.Taup

	def step_dynamics(self, dt):
		alpha = 0
		beta = 0

		self.u = np.clip(self.u, 0, 15)
		
		self.r += self.v * dt
		
		self.v += self.thrust() * dt
		self.v[0] += alpha * dt
		self.v[1] += beta * dt
		self.v[2] -= self.mass * self.g * dt

		self.q += self.w * dt
		self.w += la.inv(self.I) @ ((-tilde(self.w) @ self.I @ self.w) + self.torque())

		if self.r[2] <= 0: # ground
			self.r[2] = 0
			self.v[2] = max(0, self.v[2])

	def demo_control_law(self, r):
		hover_throttle = np.sqrt(9.81 / 4)

		K_att = 50 * la.pinv([[1,  0, -1,  0],
						      [0, -1,  0,  1], 
						  	[-10, 10,-10, 10]])

		K_rate = 35 * la.pinv([[1,  0, -1,  0],
						       [0, -1,  0,  1], 
						  	 [-10, 10,-10, 10]])

		K_q = 0.00001;

		K_zp = 0.01 * np.ones(4)
		K_zd = 0.02 * np.ones(4)

		qr = K_q * np.array([self.r[1] - r[1], r[0] - self.r[0], 0])
		att_comp = K_att @ (qr - self.q)
		rate_comp = -K_rate @ self.w
		alt_comp = K_zp * (r[2] - self.r[2])
		alt_damp = -K_zd * self.v[2]
		grav_comp = np.ones(self.u.shape) * np.sqrt(self.g / len(self.u)) * np.cos(self.q[2])

		self.u = att_comp + rate_comp + alt_comp + alt_damp + grav_comp

	def control_callback(self, msg):
		self.sensor_flags[0] = 1
		self.sensor_mag = np.array(msg.data)

	def broadcast_tf(self):
		quaternion = euler_to_quaternion(self.q[0], self.q[1], self.q[2])
		static_transformStamped = geometry_msgs.msg.TransformStamped()
		static_transformStamped.header.frame_id = "map"
		static_transformStamped.child_frame_id = "base_link1" 
		static_transformStamped.header.stamp = rospy.Time.now()
			
		static_transformStamped.transform.translation.x = self.r[0]
		static_transformStamped.transform.translation.y = self.r[1]
		static_transformStamped.transform.translation.z = self.r[2]

		static_transformStamped.transform.rotation.w = quaternion[0]
		static_transformStamped.transform.rotation.x = quaternion[1]
		static_transformStamped.transform.rotation.y = quaternion[2]
		static_transformStamped.transform.rotation.z = quaternion[3]
		self.broadcaster.sendTransform(static_transformStamped)


	def spin(self):
		bias_count = 0
		bias = np.zeros(6)

		r = [0, 0, 5]

		while not rospy.is_shutdown():
			self.demo_control_law(r)
			self.step_dynamics(1/self.hz)
			self.broadcast_tf()

			if la.norm(r - self.r) < 0.1:
				r = [0, 0, 5 * abs(np.sin(time.time())) + 2]

			self.rate.sleep()


if __name__ == '__main__':
	rospy.init_node('localization')
	sim = Rufous_Simulation()
	sim.spin()