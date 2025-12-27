import cv2 as cv

# ------------------ READ AND PLAY IMAGES AND VIDEOS ---------------------

# img = cv.imread('opencv-course/Resources/Photos/cat.jpg')

# cv.imshow('Cat', img)

# cv.waitKey(0)

# ------------------------------------------------------------------------

# capture = cv.VideoCapture('opencv-course/Resources/Videos/dog.mp4')

# while True: 
#     isTrue, frame = capture.read()
#     cv.imshow('Video',frame) # despliega una imágen 'frame' en una venta de nombre 'Video'

#     # 0xFF==ord('s') indica que con la tecla 's' se detiene el video
#     if cv.waitKey(20) &0xFF==ord('d'): 
#         break 

# capture.release()
# cv.destroyAllWindows()

# ------------------ RESIZING AND RESACLING IMAGES AND VIDEOS ---------------------

# img = cv.imread('opencv-course/Resources/Photos/cat_large2.jpg')
# cv.imshow('Cat',img)

def rescaleFrame(frame, scale=0.75):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

def changeRes(width, height):
    # this for live video
    capture.set(3,width)
    capture.set(4, height)

capture = cv.VideoCapture('opencv-course/Resources/Videos/dog.mp4')

while True: 
    isTrue, frame = capture.read()
    frame_resized = rescaleFrame(frame, scale=0.2)
        
    cv.imshow('Video',frame) # despliega una imágen 'frame' en una venta de nombre 'Video'
    cv.imshow('rescaled Video',frame_resized) # despliega una imágen 'frame' en una venta de nombre 'Video'

    # 0xFF==ord('s') indica que con la tecla 's' se detiene el video
    if cv.waitKey(20) &0xFF==ord('d'): 
        break 

capture.release()
cv.destroyAllWindows()