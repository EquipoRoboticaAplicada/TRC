import cv2 as cv

img = cv.imread('opencv-course/Resources/Photos/cat.jpg')
# cv.imshow('Cat', img)

# Converting to grayscale
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imshow('Gray', gray)

# Blur 
blur = cv.GaussianBlur(img, (15,1), cv.BORDER_DEFAULT) # matriz en impares a fuerzas
blur2 = cv.GaussianBlur(img, (1,15), cv.BORDER_DEFAULT) # matriz en impares a fuerzas
cv.imshow('blur_x', blur)
cv.imshow('blur_y', blur2)

blur = cv.GaussianBlur(img, (5,5), cv.BORDER_DEFAULT) # matriz en impares a fuerzas
cv.imshow('blur_x', blur)

# edge cascade
canny = cv.Canny(img, 10, 35)
cv.imshow('Canny edges', canny)

cv.waitKey(0)