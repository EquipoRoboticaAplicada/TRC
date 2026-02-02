import cv2 as cv
import time 

camera = cv.VideoCapture(0, cv.CAP_V4L2)
camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
camera.set(cv.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

def gen_frames():
    while True:
        ret, frame = camera.read()
        if not ret:
            break
        _, buffer = cv.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
