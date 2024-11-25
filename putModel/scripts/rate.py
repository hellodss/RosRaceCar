import cv2
import numpy as np

image = cv2.imread('src/putModel/map_picture/map.pgm')
length = image.shape[0]
width = image.shape[1]

# Define the rate of the map
length_rate = 16.7 / length
width_rate = 5.5 / width

rate =  0.5 * (length_rate + width_rate)

print(rate)
