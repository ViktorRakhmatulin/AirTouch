import io
import socket
import struct
from PIL import Image
import cv2
import numpy
import pupil_apriltags as apriltag
import time



camera_params=(4.9989302084566577e+02, 5.0320386297363052e+02, 3.2668799142880744e+02, 2.3439979484610001e+02)
tag_size = 0.0375

print('Begin')
# Start a socket listening for connections on 0.0.0.0:8000 (0.0.0.0 means
# all interfaces)
server_socket = socket.socket()
server_socket.bind(('0.0.0.0', 8000))
server_socket.listen(0)
print('Here')
# Accept a single connection and make a file-like object out of it
connection = server_socket.accept()[0].makefile('rb')
print('Before trying')
#Initializing detector
detector = apriltag.Detector(
		families = 'tag36h11',
		nthreads = 10,
		quad_decimate = 0.5,
		quad_sigma = 0,
		refine_edges = 1,
		decode_sharpening = 0.6,
		debug = 0)
try:
    while True:
        # Read the length of the image as a 32-bit unsigned int. If the
        # length is zero, quit the loop
        print('to unpack')
        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        print('I am here')
        if not image_len:
            break
        # Construct a stream to hold the image data and read the image
        # data from the connection
        image_stream = io.BytesIO()
        image_stream.write(connection.read(image_len))
        # Rewind the stream, open it as an image with PIL and do some
        # processing on it
        image_stream.seek(0)
        image = Image.open(image_stream)
        opencvImage = cv2.cvtColor(numpy.array(image), cv2.COLOR_RGB2BGR)
            #Aprilshit
        gray = cv2.cvtColor(opencvImage,cv2.COLOR_BGR2GRAY)
        #cv2.imshow("gray", gray)
        results = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)
#        print("[INFO] {} total AprilTags detected".format(len(results)))
        for r in results:
            (ptA,ptB,ptC,ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(gray, ptA, ptB, (0, 255, 0), 2)
            cv2.line(gray, ptB, ptC, (0, 255, 0), 2)
            cv2.line(gray, ptC, ptD, (0, 255, 0), 2)
            cv2.line(gray, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(gray, (cX, cY), 5, (0, 0, 255), -1)
            # draw the pose of the AprilTag
            fx, fy, cx, cy = camera_params
            K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

            rvec, _ = cv2.Rodrigues(r.pose_R)
            rvec = r.pose_R
        
            dcoeffs = numpy.zeros(5)

            opoints = numpy.float32([[1,0,0],
                             [0,-1,0],
                             [0,0,-1]]).reshape(-1,3) * tag_size

            ipoints, _ = cv2.projectPoints(opoints, rvec, r.pose_t, K, dcoeffs)
            ipoints = numpy.round(ipoints).astype(int)

            center = numpy.round(r.center).astype(int)
            center = tuple(center.ravel())
            cv2.line(gray, center, tuple(ipoints[0].ravel()), (0,0,255), 2)
            cv2.line(gray, center, tuple(ipoints[1].ravel()), (0,255,0), 2)
            cv2.line(gray, center, tuple(ipoints[2].ravel()), (255,0,0), 2)
            # draw the tag family on the image
            id_fam = str(r.tag_id)
            cv2.putText(gray, id_fam, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] tag family: {}".format(id_fam))
        cv2.imshow('Image', gray)
        image.verify()
#        print('Image is verified')
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
finally:
    connection.close()
    print('Closing')
    server_socket.close()