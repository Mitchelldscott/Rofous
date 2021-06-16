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
	def __init__(self, namespace, parent, sensor):
		rospy.init_node('tf_broadcaster')

		self.ns = namespace
		self.parent = parent
		self.rate = rospy.Rate(10)
		self.sensor = sensor
		self.broadcaster = tf.TransformBroadcaster()

		self.sub = rospy.Subscriber(self.sensor, PoseStamped, self.motionCallback)

		self.measurement = np.zeros(6)

	def motionCallback(self, data):
		pose = data.pose.position
		q = data.pose.orientation
		
		self.broadcaster.sendTransform((pose.x, pose.y, pose.z), (q.x, q.y, q.z, q.w), rospy.Time.now(), 
			f'/odometry', self.parent)
		

if __name__ == '__main__':
	
	broadcaster = tf_Broadcaster(sys.argv[1], sys.argv[2], sys.argv[3])    
	rospy.spin()