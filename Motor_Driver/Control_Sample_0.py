import board
import busio
import adafruit_pca9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

pca.frequency = 600

m1 = pca.channels[0]
m2 = pca.channels[4]

motors = [m1, m2]

for motor in motors:
	throttle = input('Enter Initial Throttle: ')
	motor.duty_cycle = int(throttle)
	while 1:
		print(f'Set Motor to {throttle}')
		throttle = input('Enter Updated Throttle: ')
		if throttle == 'q':
			break
		motor.duty_cycle = int(throttle)
	motor.duty_cycle = 0
