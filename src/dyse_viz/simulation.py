#!/usr/bin/env python3

import tf
import time
import rospy
import tf2_ros
import numpy as np
import std_msgs.msg
import sensor_msgs.msg
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

def euler_to_quaternion(roll, pitch, yaw):
	# yaw = np.rad2deg(yaw)
	# pitch = np.rad2deg(pitch)
	# roll = np.rad2deg(roll)
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qw, qx, qy, qz]

def broadcast_tf(r, q, parent, child, broadcaster):
	quat = euler_to_quaternion(q[0], q[1], q[2])
	static_transformStamped = geometry_msgs.msg.TransformStamped()
	static_transformStamped.header.frame_id = parent
	static_transformStamped.child_frame_id = child
	static_transformStamped.header.stamp = rospy.Time.now()
	static_transformStamped.transform.translation.x = r[0]
	static_transformStamped.transform.translation.y = r[1]
	static_transformStamped.transform.translation.z = r[2]
	static_transformStamped.transform.rotation.w = quat[0]
	static_transformStamped.transform.rotation.x = quat[1]
	static_transformStamped.transform.rotation.y = quat[2]
	static_transformStamped.transform.rotation.z = quat[3]
	broadcaster.sendTransform(static_transformStamped)


class Rufous_Simulation():
	def __init__(self):
		#		States
		self.r = np.zeros(3)				# postion
		self.v = np.zeros(3)				# velocity
		self.q = np.zeros(3)				# attitude
		self.w = np.zeros(3)				# angular rate
		self.u = np.zeros(4)				# control output, input to the plant
		self.rr = np.zeros(3)				# attitude reference
		self.qr = np.zeros(3)				# attitude reference
		self.Tp = np.zeros(3)				# thrust from propellers
		self.Taup = np.zeros(3)				# torque from propellers


		#		Parameters
		self.l = 0.23  								# wing length (center to prop rot axis)
		self.g = 9.81								# gravitational constant
		self.Kt = 0.02								# thrust constant
		self.mass = 0.025							# quadcopter mass
		self.I = np.array([[3.2132169,  0,  0], 	# Inertia matrix (from collin)
							[0, 5.0362409,  0], 
							[0, 0, 7.22155076]]) * 1e-3

		#		ROS setup
		self.hz = 100
		self.rate = rospy.Rate(self.hz)
		self.prop_pub = rospy.Publisher('prop_speeds', std_msgs.msg.Float64MultiArray, queue_size=1)
		self.pos_pub = rospy.Publisher('position', std_msgs.msg.Float64MultiArray, queue_size=1)
		self.vel_pub = rospy.Publisher('velocity', std_msgs.msg.Float64MultiArray, queue_size=1)
		self.att_pub = rospy.Publisher('attitude', std_msgs.msg.Float64MultiArray, queue_size=1)
		self.ang_pub = rospy.Publisher('angular_rate', std_msgs.msg.Float64MultiArray, queue_size=1)
		self.ctl_pub = rospy.Publisher('controllers', std_msgs.msg.Float64MultiArray, queue_size=1)
		self.broadcaster_b1 = tf2_ros.StaticTransformBroadcaster()
		self.broadcaster_b2 = tf2_ros.StaticTransformBroadcaster()
		self.broadcaster_ref = tf2_ros.StaticTransformBroadcaster()

	def thrust(self):
		"""
			  Calculates the thrust from an input u.
			Must set self.u before calling.
			
		"""
		Rnb = DCM(-self.q)
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
		self.q = np.fmod(self.q, 2 * np.pi)

		# print(self.w)
		# print(self.q, self.u)

		if la.norm(self.w) > 1e5:		# instability classifier
			rospy.logerr("Unstable result detected")
			rospy.logerr(f"Large Omega: {self.w} {self.torque()}")

		if self.r[2] <= 0: # ground
			self.r[2] = 0
			self.v[2] = max(0, self.v[2])

	def demo_control_law(self, update_ref=False):
		hover_throttle = np.sqrt(self.g * self.mass / (self.Kt * len(self.u))) * abs(np.cos(self.q[0]) * np.cos(self.q[1]))

		K_att = 0.075 * la.pinv([[1,  0, -1,  0],
						         [0, -1,  0,  1], 
						  	    [-1,  1, -1,  1]])

		K_rate = 0.1 * la.pinv([[1,  0, -1,  0],
						        [0, -1,  0,  1], 
						  	   [-1,  1, -1,  1]])

		K_q = 0.5

		K_zp = 4 * np.ones(4)
		K_zd = 5 * np.ones(4)

		if update_ref:
			self.qr = K_q * np.array([self.r[1] - self.rr[1], self.rr[0] - self.r[0], 0])

		att_comp = K_att @ (self.qr - self.q)
		att_damp = -K_rate @ self.w
		alt_comp = K_zp * (self.rr[2] - self.r[2])
		vel_damp = -K_zd * self.v[2]
		grav_comp = np.ones(self.u.shape) * hover_throttle 
		
		self.u = att_comp + att_damp + grav_comp + alt_comp + vel_damp

	def control_callback(self, msg):
		self.sensor_flags[0] = 1
		self.sensor_mag = np.array(msg.data)

	def broadcast_data(self):
		broadcast_tf(self.r, [0, 0, 0], "map", "body_fixed", self.broadcaster_b1)
		broadcast_tf([0, 0, 0], self.q, "body_fixed", "base_link1", self.broadcaster_b2)
		broadcast_tf(self.rr, self.qr, "map", "reference", self.broadcaster_ref)

		msg = std_msgs.msg.Float64MultiArray(data=self.u)
		self.prop_pub.publish(msg)

		msg.data = self.r
		self.pos_pub.publish(msg)

		msg.data = self.v
		self.vel_pub.publish(msg)

		msg.data = self.q
		self.att_pub.publish(msg)

		msg.data = self.w
		self.ang_pub.publish(msg)


	def spin(self):
		bias_count = 0
		bias = np.zeros(6)

		self.r = np.array([0.0, 0.0, 1.0])
		self.q = np.array([0.0, -0.0, 1.0])
		self.w = np.array([0.0, -0.0, 0.0])

		loop_ctr = 0

		self.rr = np.array([1, 0, 1])

		for i in range(150):
			self.rate.sleep()

		update_ref = False
		while not rospy.is_shutdown():
	
			if loop_ctr % 10 == 0:
				update_ref = True
				self.broadcast_data()

			self.demo_control_law(update_ref=update_ref)
			update_ref = False
			# for i in range(10): # 10 steps per control law
			self.step_dynamics(1/self.hz)

			# if la.norm(self.rr - self.r) < 0.1:
			# 	self.rr = np.array([0, 0, 1 + abs(np.sin(time.time()))])

			self.rate.sleep()
			loop_ctr += 1


if __name__ == '__main__':
	rospy.init_node('simulator')
	sim = Rufous_Simulation()
	sim.spin()