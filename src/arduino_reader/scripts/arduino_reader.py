#!/usr/bin/env python3
"""
	Arduino reader script
	Author : Mitchell Scott
		- misc4432@colorado.edu
	Project : Rofous
"""
import os
import sys
import time
import rospy
import serial
import numpy as np
import traceback as tb
from std_msgs.msg import String, Float64
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Quaternion



class ArduinoListener:
	def __init__(self, baud, port):
		"""
		  A node that listens to an arduino sending one-liners.
		The first element of each msg is the id, the second is the 
		type of message. All messages are comma seperated sequences.
		There are a few optional parameters that can change 
		how you publish the data.
		"""
		
		self.port = port
		self.device = None
		self.baudrate = baud
		self.connected = False

		self.dtype_LUT = {
			'0' : String,
			'1' : Float64,
			'2' : PoseStamped,
		}

		self.publishers = {}

		self.tryConnect()

		rospy.init_node(f'Serial_Publisher', anonymous=True)

	def tryConnect(self):
		"""
		  Try to connect with the device.

		Returns:
		  status: bool - True if successful
		"""
		try:
			self.device = serial.Serial(self.port, self.baudrate, timeout=1)
			self.device.flush()
			self.log('Device Connected: Reading...')
			self.connected = True
			return True 

		except Exception as e:
			self.log('Error Detecting Device')
			self.log(e)
			self.connected = False
			return False

	def log(self, text):
		"""
		  Wrapper to print a value through /rosout.

			Params:
			  text: value - prints this
		"""
		rospy.loginfo(f'[Arduino_Reader]: {text}')

	def initPublisher(self, topic, msgType):
		"""
		  Create a new publisher.

			Params:
			  topic: string - destination for the publisher
			  msgType: string - the datatype of the msg
		"""
		if topic == 'SUDO':
			if msgType == -1:
				self.log(f'Session Terminated by {topic} on Device')
				exit(0)

		msgType = self.dtype_LUT[msgType]
		self.publishers[topic] = rospy.Publisher(topic, msgType, queue_size=1)


	def parseMsg(self, topic, msgType, timestamp, data=None):
		"""
		  Create a new msg to publish.

			Params:
				topic: string - destination for the publisher
				msgType: string - the datatype of the msg
				data: ??? - the data to fill msg

			Returns:
			  msg: ??? - the msg to send
		"""
		msg = None
		if msgType == '0':
			for string in data:
				msg = String(string)
				self.log(f'String: {string}')

		elif msgType == '1':
			for fl in data:
				msg = Float64(float(fl))

		elif msgType == '2':
			msg = PoseStamped()
			msg.header.frame_id = topic
			msg.header.stamp = timestamp
			msg.pose.position = Point(float(data[0]), float(data[1]), float(data[2]))
			q = quaternion_from_euler(float(data[3]), float(data[4]), float(data[5]))
			msg.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
		
		return msg

	def spin(self):
		"""
		  Read a line from the serial port and publish it
		to a topic given the mode. The mode is the first 4 bytes
		of the msg.

			ROS Publishers:
			  - /namespace/string : String
			  - /namespace/float : Float
			  - /namespace/floatMulti : FloatMultiArray
		"""
		try:
			while not rospy.is_shutdown():
				if not self.connected:
					self.tryConnect()

				elif self.device.in_waiting:
					msg = self.device.readline().decode().rstrip().split(',')

					topic = None
					msgType = -1
					timestamp = 0.0
					msgLen = len(msg)

					if len(msg) > 3:
						topic = msg[0]
						msgType = msg[1]
						timestamp = rospy.Time.from_sec(float(msg[2]))
						data = msg[3:]

					elif len(msg) > 1:
						topic = msg[0]
						
					if topic is None:
						continue

					if not topic in self.publishers:
						self.initPublisher(topic, msgType)

					pub = self.publishers[topic]
					msg = self.parseMsg(topic, msgType, timestamp, msg[3:])
					pub.publish(msg)

		except Exception as e:
			self.log('Possible I/O Error: Restarting...')
			time.sleep(2)
			self.spin()

if __name__=='__main__':
	try:
		arduino_relay = ArduinoListener(int(sys.argv[1]), sys.argv[2])
		arduino_relay.spin()

	except KeyboardInterrupt:
		rospy.loginfo('[Arduino_Reader]: Exiting ...')

	except Exception as e:
		exc_type, exc_value, exc_traceback = sys.exc_info()
		tb.print_exc()