from picamera.array import PiRGBArray
from picamera.array import PiArrayOutput
from picamera import PiCamera
import time
import cv2
import pupil_apriltags as apriltag
import numpy as np

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
counter = 0
step = 10

#Initializing detector
detector = apriltag.Detector(
		families = 'tag36h11',
		nthreads = 1,
		quad_decimate = 1.0,
		quad_sigma = 0.2,
		refine_edges = 1,
		decode_sharpening = 0.25,
		debug = 0)

# allow the camera to warmup
time.sleep(0.1)

#record video
fourcc =cv2.VideoWriter_fourcc(*'XVID')
video_writer = cv2.VideoWriter('Test_ply.avi',fourcc,20.0,(640,480))
    
	
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture,format = "bgr",use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
 # show the frame
    print(image.max())
    image = np.array(image,dtype = np.uint8)
    key = cv2.waitKey(1) & 0xFF
    
    #Aprilshit
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    #cv2.imshow("gray", gray)
    blur = cv2.GaussianBlur(gray,(3,3),0)
    clahe_parameters = cv2.createCLAHE(clipLimit = 10.0, tileGridSize = (8,8))
    clahe = clahe_parameters.apply(blur)
    cv2.imshow("clahe", gray)
    print("[INFO] detecting AprilTags...")
    results = detector.detect(gray, estimate_tag_pose=True, camera_params=(4.9989302084566577e+02, 5.0320386297363052e+02, 3.2668799142880744e+02, 2.3439979484610001e+02), tag_size=0.0375)
    print("[INFO] {} total AprilTags detected".format(len(results)))
    for r in results:
        (ptA,ptB,ptC,ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0										, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        cv2.line(image, (cX, cY), (cX + int(r.pose_R[0][0]*100), cY + int(r.pose_R[0][1]*100)), (255, 0, 0), 2)
        cv2.line(image, (cX, cY), (cX + int(r.pose_R[1][0]*100), cY + int(r.pose_R[1][1]*100)), (0, 255, 0), 2)
        cv2.line(image, (cX, cY), (cX + int(r.pose_R[2][0]*100), cY + int(r.pose_R[2][1]*100)), (0, 0, 255), 2)
        # draw the tag family on the image
        id_fam = str(r.tag_id)
        cv2.putText(image, id_fam, (ptA[0], ptA[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("[INFO] tag family: {}".format(id_fam))
    cv2.imshow("Frame",image)
    if(counter % 10 == 0):
        cv2.imwrite("images/"+str(counter//step)+"_test_ply.png",image)
#    vid_frame = cv2.flip(image,0)
    counter +=1
    video_writer.write(image)
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
	
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        video_writer.release()
        cv2.destroyAllWindows()
        break
