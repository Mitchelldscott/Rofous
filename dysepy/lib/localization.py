#!/usr/bin/env python3

import math
import rospy
import tf
import numpy as np

import tf2_ros
import geometry_msgs.msg

from scipy import signal

from ahrs.filters import Complementary
from std_msgs.msg import String, Float64, Float64MultiArray
from tf.transformations import euler_from_quaternion


class IMU_Odometry():
	def __init__(self):
		self.complementary = Complementary(gain=0.45)

		self.sensor_mag = np.zeros(3)
		self.sensor_gyro = np.zeros(3)
		self.sensor_accel = np.zeros(3)
		self.sensor_flags = np.zeros(3)

		self.euler_angles = np.zeros(3)
		self.omega = np.zeros(3)
		self.omega_history = np.zeros((50,3))
		self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
		self.q_prop = np.array([1.0, 0.0, 0.0, 0.0])

		self.broadcaster = tf2_ros.StaticTransformBroadcaster()

		self.mag_sub = rospy.Subscriber('/mag_raw', Float64MultiArray, self.mag_callback)
		self.gyro_sub = rospy.Subscriber('/gyro_raw', Float64MultiArray, self.gyro_callback)
		self.accel_sub = rospy.Subscriber('/accel_raw', Float64MultiArray, self.accel_callback)

		self.state_publisher = rospy.Publisher('/state', Float64MultiArray, queue_size=1)
		self.omega_publisher = rospy.Publisher('/omega_raw', Float64MultiArray, queue_size=1)

		self.valid_bias = False
		self.state_bias = np.zeros(6)

		rate = 384
		self.dt = 1 / rate
		self.rate = rospy.Rate(rate)
		self.complementary.DT = 1 / rate

		self.fc = 600  # Cut-off frequency of the filter
		self.w = self.fc / (rate / 2) # Normalize the frequency
		self.sos = signal.butter(2, self.w, btype='low', fs=rate, output='sos')


	def mag_callback(self, msg):
		self.sensor_flags[0] = 1
		self.sensor_mag = np.array(msg.data)
	
	def gyro_callback(self, msg):
		self.sensor_flags[1] = 1
		self.sensor_gyro = np.array(msg.data) #* np.pi / 180.0
	
	def accel_callback(self, msg):
		self.sensor_flags[2] = 1
		self.sensor_accel = np.array(msg.data) / 9.81

	def broadcast_tf(self):
		static_transformStamped = geometry_msgs.msg.TransformStamped()
		static_transformStamped.header.frame_id = "map"
		static_transformStamped.child_frame_id = "base_link1" 
		static_transformStamped.header.stamp = rospy.Time.now()
			
		static_transformStamped.transform.translation.x = 0.0
		static_transformStamped.transform.translation.y = 0.0
		static_transformStamped.transform.translation.z = 0.0

		static_transformStamped.transform.rotation.w = self.quaternion[0]
		static_transformStamped.transform.rotation.x = self.quaternion[1]
		static_transformStamped.transform.rotation.y = self.quaternion[2]
		static_transformStamped.transform.rotation.z = self.quaternion[3]
		self.broadcaster.sendTransform(static_transformStamped)

	def broadcast_state(self):
		msg = Float64MultiArray()
		theta, phi, psi = self.euler_angles
		theta_dot, phi_dot, psi_dot = self.omega

		msg.data = np.array([theta, theta_dot, phi, phi_dot, psi, psi_dot])
		self.state_publisher.publish(msg)
		
		msg.data = self.omega_history[-1]
		self.omega_publisher.publish(msg)

	def update_state(self, q):
		euler_angles = (np.flip(euler_from_quaternion(q)) - self.state_bias[::2])
		if euler_angles[2] > np.pi:
			euler_angles[2] -= 2 * np.pi

		elif euler_angles[2] < -np.pi:
			euler_angles[2] += 2 * np.pi

		self.omega_history[:-1] = self.omega_history[1:]
		self.omega_history[-1] = self.sensor_gyro - self.state_bias[1::2]
		self.quaternion = q
		self.euler_angles = euler_angles

		self.omega[0] = signal.sosfilt(self.sos, self.omega_history[:,0])[-1] #self.sensor_gyro[0] - self.state_bias[1]#
		self.omega[1] = signal.sosfilt(self.sos, self.omega_history[:,1])[-1] #self.sensor_gyro[1] - self.state_bias[3]#
		self.omega[2] = signal.sosfilt(self.sos, self.omega_history[:,2])[-1] #self.sensor_gyro[2] - self.state_bias[5]#

	def complementary_filter(self):
		try:
			new_quaternion = self.complementary.update(self.quaternion, 
													mag=self.sensor_mag,
													gyr=self.sensor_gyro, 
													acc=self.sensor_accel)

			return new_quaternion

		except ValueError as e:
			print(e)
			print('bad value dropping frame')
			return self.quaternion


	def spin(self):
		bias_count = 0
		bias = np.zeros(6)

		while not rospy.is_shutdown():
			if np.sum(self.sensor_flags) > 0:
				self.sensor_flags *= 0
				q = self.complementary_filter()
				self.update_state(q)

				if self.valid_bias:
					self.broadcast_state()
					self.broadcast_tf()

				elif not math.isnan(self.euler_angles[2]):
					bias += np.array([0, self.sensor_gyro[0], 0, self.sensor_gyro[1], self.euler_angles[2], self.sensor_gyro[2]])
					bias_count += 1
					self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
					self.broadcast_tf()
					if bias_count >= 50:
						self.valid_bias = True
						self.state_bias = bias / bias_count
						print(f'Euler angle Bias {self.state_bias}')
				
				else:
					self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
					self.broadcast_tf()

			else:
				self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
				self.broadcast_tf()

			self.rate.sleep()


if __name__ == '__main__':
	rospy.init_node('localization')
	odom = IMU_Odometry()
	odom.spin()