import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

def load_images(path, num_images):
	images = []
	for i in range(num_images):
		images.append(cv.imread(path + f'{i}.jpg'))
	print(f'{len(images)} Images loaded successfully!')
	return images


images = load_images('field-images/img', 63)
for i, image in enumerate(images):
	print(image.shape)
	cv.imshow(f'Image {i}', image)
	cv.waitKey()
	
	
"""	with open('field-images/field-poi-labels.txt', 'a+') as f:
	p0 = input('1: ')
	p1 = input('2: ')
	p2 = input('3: ')
	p3 = input('4: ')
	c2 = input('c1: ')
	c1 = input('c2: ')
	poi = input('mp: ')
		f.write(f'{label[0]},{label[1]},{label[2]},{label[3]},{label[4]},{label[5]},{label[6]}\n')"""
