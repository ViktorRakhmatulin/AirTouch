import io
import socket
import struct
from PIL import Image
import cv2
import numpy
import pupil_apriltags as apriltag
import time
from scipy.spatial.transform import Rotation as R
import transforms3d

camera_params = (4.9989302084566577e+02, 5.0320386297363052e+02,
                 3.2668799142880744e+02, 2.3439979484610001e+02)
tag_size = 0.0375

UDP_IP = "127.0.0.1"
UDP_PORT = 5065
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

last = []
cap = cv2.VideoCapture(0)
# at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9') )
# Initializing detector
detector = apriltag.Detector(
    families='tag36h11',
    nthreads=1,
    quad_decimate=0.5,
    quad_sigma=0,
    refine_edges=1,
    decode_sharpening=0.6,
    debug=0)

i = 0
while (1):
    # Get image
    ret, frame = cap.read()
    # Detect button
    k=cv2.waitKey(1)
    if k==27:
        break
    elif k==ord('s'):
        i+=1
    # Detect apriltag
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image = frame
    results = detector.detect(
            gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)
#        print("[INFO] {} total AprilTags detected".format(len(results)))
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # extract camera parameters
        fx, fy, cx, cy = camera_params
        # find camera matrix
        K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
        # extract rotation and translation vectors
        # rvec, _ = cv2.Rodrigues(r.pose_R)
        rvec = r.pose_R

        dcoeffs = numpy.zeros(5)
        # find object points of the tag
        opoints = numpy.float32([[1, 0, 0],
                                    [0, -1, 0],
                                    [0, 0, -1]]).reshape(-1, 3) * tag_size
        # project object points to image plane
        ipoints, _ = cv2.projectPoints(opoints, rvec, r.pose_t, K, dcoeffs)
        ipoints = numpy.round(ipoints).astype(int)
        # find distance between camera and tag

        # april_distance = tag_size * fx / \
        #    numpy.linalg.norm(ipoints[0] - ipoints[1])
        # distance = round(april_distance, 2)
        # cv2.putText(gray, str(distance), (ptA[0], ptA[1] - 30),
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # find the center of the tag
        center = numpy.round(r.center).astype(int)
        center = tuple(center.ravel())
        # draw the axis of the tag
        cv2.line(image, center, tuple(ipoints[0].ravel()), (0, 0, 255), 2)
        cv2.line(image, center, tuple(ipoints[1].ravel()), (0, 255, 0), 2)
        cv2.line(image, center, tuple(ipoints[2].ravel()), (255, 0, 0), 2)
        # draw the tag family on the image
        id_fam = str(r.tag_id)
        cv2.putText(image, id_fam, (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # print the information about the tag: id, rotation, translation, center, corners, distance
        # print("[INFO] tag id: {}".format(id_fam))
        # print("Rotation: {}".format(r.pose_R))
        # print("Translation: {}".format(r.pose_t))
        # print("Center: {}".format(r.center))
        # print("Corners: {}".format(r.corners))
        # print("Distance: {}".format(distance))
        # print("")
        # store rotation matrix and translation vector, translate rotation matrix to quaternion, make string and send it to the client
        # and send it to the client
        rotation = r.pose_R
        rotation = numpy.round(rotation, 3)
        rot_quat = transforms3d.quaternions.mat2quat(rotation)
        rot_quat = numpy.round(rot_quat, 3)
        translation = r.pose_t
        translation = numpy.round(translation, 3)
        translation = translation.ravel()
        string = str(id_fam) + ',' + str(rot_quat[0]) + ',' + str(rot_quat[1]) + ',' + str(rot_quat[2]) + ',' + str(rot_quat[3]) + ',' + str(translation[0]) + ',' + str(translation[1]) + ',' + str(translation[2])
        # print(string)
        # print("")
        send_string = string.encode('utf-8')
        sock.sendto(send_string, (UDP_IP, UDP_PORT))

    
#        print('Image is verified')
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
    cv2.imshow('capture', frame)