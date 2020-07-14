import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

def load_images(path, num_images):
	images = []
	for i in range(num_images):
		images.append(cv.imread(path + f'{i}.jpg'))
	print(f'{len(images)} Images loaded successfully!')
	return images


images = load_images('feild-images/img', 65)
plt.imshow(images[1])