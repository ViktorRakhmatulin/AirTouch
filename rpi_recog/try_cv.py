from picamera import PiCamera
from time import sleep
import cv2 as cv

print(cv.__version__)

camera = PiCamera()
camera.resolution = (640,480)
camera.start_preview()
sleep(2)

cap = cv.VideoCapture(usePiCamera = True)
if not cap.isOpened():
    print('Cannot open the camera')
    exit()
while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame(stream med?). Exiting...")
        break
    
    gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    cv.imshow('frame',gray)
    if cv.waitkey(1) == ord('q'):
        break
cap.release()
cv.destroyAllWindows()