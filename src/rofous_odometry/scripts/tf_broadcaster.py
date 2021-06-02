#!/usr/bin/env python3
"""
	Rofous tf broadcast script
	Author : Mitchell Scott
		- misc4432@colorado.edu
	Project : Rofous
""" 
import tf
import sys
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion, Point

class tf_Broadcaster:
	def __init__(self, namespace, parent):
		rospy.init_node('tf_broadcaster')

		self.ns = namespace
		self.parent = parent
		self.rate = rospy.Rate(10)
		self.broadcaster = tf.TransformBroadcaster()
		self.motion_topic = '/Rofous/IMU'

		self.sub = rospy.Subscriber(self.motion_topic, PoseStamped, self.motionCallback)

		self.measurement = np.zeros(6)

	def motionCallback(self, data):
		
		self.broadcaster.sendTransform((data.pose.position.x, data.pose.position.y, data.pose.position.z)
		, (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w), 
			rospy.Time.now(),
			"/Rofous/odom", "/world")
		

if __name__ == '__main__':
	
	broadcaster = tf_Broadcaster(sys.argv[1], sys.argv[2])    
	rospy.spin()