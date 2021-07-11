#!/usr/bin/env python3
"""
	Arduino reader script
	Author : Mitchell Scott
		- misc4432@colorado.edu
	Project : Rofous
"""
import os
import sys
import yaml
import time
import rospy
import serial
import traceback
import numpy as np
import traceback as tb
from std_msgs.msg import String, Float64
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Quaternion


dType_LUT = {
	'0' : String,
	'1' : Float64,
	'2' : PoseStamped
}

class SerialListener:
	def __init__(self, config):
		"""
		  A node that listens to an arduino sending one-liners.
		The first element of each msg is the id, the second is the 
		type of message. All messages are comma seperated sequences.
		There are a few optional parameters that can change 
		how you publish the data.
		"""
		self.nPorts = 0
		self.config = None
		self.device = None
		self.loadDevice(config)
		self.tryConnect()

		rospy.init_node(self.config['WhoAmI'], anonymous=True)

	def loadDevice(self, config):
		"""
		  Try to load the conf file for a device.

		"""
		kill = 0
		with open(config, 'r') as stream:
			self.config = yaml.safe_load(stream)

		self.log(self.config)
		if not 'WhoAmI' in self.config:
			self.config['WhoAmI'] = 'Serial_Reader'
		if not 'Baudrate' in self.config:
			kill = 1
			self.log('Missing Baudrate')
		if not 'Ports' in self.config:
			kill = 1
			self.log('Missing Ports')
		else:
			 self.nPorts = len(self.config['Ports'])
		if not 'Publishers' in self.config:
			self.config['Publishers'] = {}

		if kill:
			self.log('Check your config file')
			sys.exit(0)

	def tryConnect(self, alt=0):
		"""
		  Try to connect with the device.

		Returns:
		  status: bool - True if successful
		"""
		try:
			self.device = serial.Serial(self.config['Ports'][alt], self.config['Baudrate'], timeout=self.config['Timeout'])
			self.device.flush()
			self.log('Device Connected: Reading...')
			self.connected = True
			return True 

		except Exception as e:
			self.log(f'Error Detecting Device at {self.config["Ports"][alt]}')
			self.log(e)
			time.sleep(5)
			self.connected = False
			if self.nPorts > alt + 1:
				if self.device:
					self.device.close()
				return self.tryConnect(alt=alt+1)
			return False

	def writeBack(self, mesg):
		"""
		  Write a message back to the arduino
		"""
		self.device.write(mesg)

	def log(self, text):
		"""
		  Wrapper to print a value through /rosout.

			Params:
			  text: value - prints this
		"""
		rospy.loginfo(f'[{self.config["WhoAmI"]}]: {text}')

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

		msgType = dType_LUT[msgType]
		self.config['Publishers'][topic] = rospy.Publisher(topic, msgType, queue_size=1)


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
				if topic == 'whoami':
					self.log(f'connecting to: {string} @ timestamp')
					self.writeBack('1101')

		elif msgType == '1':
			for fl in data:
				msg = Float64(float(fl))

		elif msgType == '2':
			msg = PoseStamped()
			msg.header.stamp = timestamp
			msg.header.frame_id = self.config['WhoAmI']
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
			  - /***/string : String
			  - /***/float : Float
			  - /***/PoseStamped : PoseStamped
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
						
					if topic is None:
						continue

					if not topic in self.config['Publishers']:
						self.initPublisher(topic, msgType)

					pub = self.config['Publishers'][topic]
					msg = self.parseMsg(topic, msgType, timestamp, msg[3:])
					pub.publish(msg)

		except Exception as e:
			traceback.print_exc()
			self.log(e)
			self.log('Possible I/O Error: Restarting...')
			self.device.close()
			time.sleep(2)
			self.tryConnect()
			self.spin()

if __name__=='__main__':
	try:
		arduino_relay = SerialListener(sys.argv[1])
		arduino_relay.spin()

	except KeyboardInterrupt:
		rospy.loginfo('[Serial_Reader]: Exiting ...')

	except Exception as e:
		exc_type, exc_value, exc_traceback = sys.exc_info()
		tb.print_exc()