import sys
import time
from time import sleep
from adafruit_servokit import ServoKit

def initialize_all(m1, m2, m3, m4):
	print('Initializing Motors... Standby')
	m1.angle = 0
	m2.angle = 0
	m3.angle = 0
	m4.angle = 0
	print('All Motors Successfully Initialized')

def quick_run(motor):
	motor.angle = 60
	sleep(1)
	motor.angle = 100
	sleep(1)
	motor.angle = 0
	sleep(1)

def quick_run_all(motors):
	for i, motor in enumerate(motors):
		quick_run(motor)
		print(f'Round {i} Complete')

def min2max(motor):
	for i in range(0, 180, 10):
		print(f'Running at: {i}')
		motor.angle = i
		sleep(.25)
	motor.angle = 0

def min2max_all(motors):
	for i, motor in enumerate(motors):
		min2max(motor)
		print(f'Round {i} Complete')

def onoff_all(motors):
	start = time.time()
	motors[0].angle = 100
	motors[1].angle = 100
	motors[2].angle = 100
	motors[3].angle = 100
	print(f'Startup Time: {time.time() - start}')
	sleep(2)
	start = time.time()
	motors[0].angle = 0
	motors[1].angle = 0
	motors[2].angle = 0
	motors[3].angle = 0
	print(f'Shutoff Time: {time.time() - start}')

def lift_off(motors):
	speed = input('Enter Thrust for lift off: ')
	print('Prepare for Lift-off!')
	for motor in motors:
		motor.angle = speed
	while speed > 0:
		speed -= 20
		for motor in motors:
			motor.angle = speed
		sleep(1)
	print(f'Take-off Sequence Complete!')
	print(f'Speed at {speed}')

def manned_flight(motors):
	speeds = input('Set Speeds: ')
	while not 'q' in speeds:
		for i,speed in enumerate(speeds.split(' ')):
			motors[i].angle = int(speed)
		speeds = input('Set Speeds: ')
	speed = 60
	while speed > 0:
		speed -= 20
		for motor in motors:
			motor.angle = speed
		sleep(1)
	print('Flight Terminated')

def main(tests):
	print('Running Setup:...')
	kit = ServoKit(channels=16)
	m1 = kit.servo[0]
	m2 = kit.servo[1]
	m3 = kit.servo[2]
	m4 = kit.servo[3]
	print('Setup Complete')

	initialize_all(m1, m2, m3, m4)
	sleep(1)

	for test in tests:
		if test == 'quickrun':
			quick_run_all([m1, m2,  m3, m4])
		if test == 'min2max':
			min2max_all([m1, m2, m3, m4])
		if test == 'onofftimer':
			onoff_all([m1, m2, m3, m4])
		if test == 'liftoff':
			lift_off([m1, m2, m3, m4])
		if test == 'manned':
			manned_flight([m1, m2, m3, m4])

	print('Testing Complete')


if __name__ == '__main__':
	tests = sys.argv
	main(tests)
