import cv2
import numpy as np


# Convert image to GrayScale
image = cv2.imread('test_image.jpg')
lane_image = np.copy(image)
gray = cv2.cvtColor(lane_image,cv2.COLOR_RGB2GRAY)


# Noise Filtering (gaussian blur)
blur = cv2.GaussianBlur(gray,(5,5),0)

# Edge Detection
canny = cv2.Canny(blur,50,150)

cv2.imshow('result',canny)
cv2.waitKey(0)