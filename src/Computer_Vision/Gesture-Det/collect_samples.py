import os
import time
from picamera import PiCamera as PiCam

ME = '[Collector]'
IMG_EXT = 'Hand-Sample-'

img_ctr = len([f for f in os.listdir('Data') if IMG_EXT in f])

camera = PiCam()
camera.start_preview()

for i in range(100):
	print(f'{ME} Capturing {img_ctr + i}')
	camera.capture(f'Data/{IMG_EXT}{img_ctr+i}.jpg')
	time.sleep(1)

print(f'{ME} Finished Capturing 100 images')
