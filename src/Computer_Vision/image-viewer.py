import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

def load_data(path, num_images):
	images = []
	for i in range(num_images):
		images.append(cv.imread(path + f'{i}.jpg'))
	print(f'{len(images)} Images loaded successfully!')
	with open('field-images-v1/field-poi-labels.txt', 'r') as f:
		labels = f.read().split('\n')
	return images, labels


images, labels = load_data('field-images-v1/img', 630)
for i, image in enumerate(images):
	for point in labels[i +1].split(':'):
		if point == '':
			continue
		p_actual = point.split(',')
		p = (int(p_actual[0]), int(p_actual[1]))
		cv.circle(image, p, 10, color=3)
	cv.imshow(f'Image {i}', image)
	cv.waitKey()

  