import cv2 as cv
import numpy as np
from read import rescaleFrame

blank = np.zeros((500,500,3), dtype='uint8')
# img = cv.imread('opencv-course\Resources\Photos\cat.jpg')
# cv.imshow('Blank',blank)

# Paint the image a certain colour 
blank[200:300, 300:400] = 0,0,255
# cv.imshow('Green',blank)

# Draw a rectangle
cv.rectangle(blank, (0,0), (blank.shape[1]//2, blank.shape[0]//2), (0,250,0), thickness=-1)
# cv.imshow('Rectangle', blank)

# Draw circle
cv.circle(blank, (blank.shape[1]//2, blank.shape[0]//2), 80, (250,0,0), thickness=2)
# cv.imshow('Circle', blank)

# Draw a line
cv.line(blank, (100,250), (300,400), (250,250,250), thickness=2)
# cv.imshow('line', blank)

# Write text 
cv.putText(blank, 'Hello World!', (255,255), cv.FONT_HERSHEY_TRIPLEX, 1.0, (0,250,0), thickness=1)
# cv.imshow('text', blank) 

cv.waitKey(0)