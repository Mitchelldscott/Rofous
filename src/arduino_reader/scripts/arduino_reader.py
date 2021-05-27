"""
	Arduino reader script
	Author : Mitchell Scott
		- misc4432@colorado.edu
	Project : Humming Bird
"""
import serial

def main():
	filename = None
	data = []

	device = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
	device.flush()

	print('[Reader] Connected to device!')

	while True:
		if device.in_waiting > 0:
			if filename is None:
				filename = 'data/' + device.readline().decode('ascii').rstrip() + '.csv'
				print('[Reader] Data Recieved')
				with open(filename, '+a') as f:
					f.write('Time,Xaccel,Yaccel,Zaccel,Xvel,Yvel,Zvel,PHIvel,THETAvel,PSIvel,X,Y,Z,PHI,THETA,PSI\n')
			else:
				msg = device.readline().decode('utf-8').rstrip()
				if msg == 'Kill':
					print('[Reader] Test Terminated')
					exit(0)
				with open(filename, '+a') as f:
					f.write(msg + '\n')

if __name__=='__main__':
	try:
		print('[Reader] Dont forget to clear the existing samples')
		main()
	except KeyboardInterrupt:
		print('[User Kill] Exiting ...')