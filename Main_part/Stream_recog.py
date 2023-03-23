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


camera_params = (506.19083684, 508.36108854,
                 317.93111342, 243.12403806)
tag_size = 0.0375

UDP_IP = "127.0.0.1"
UDP_PORT = 5065
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

last = []

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
# Initializing detector
detector = apriltag.Detector(
    families='tag36h11',
    nthreads=10,
    quad_decimate=0.5,
    quad_sigma=0,
    refine_edges=1,
    decode_sharpening=0.6,
    debug=0)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4',fourcc,30,(640,480))
i = 0
try:
    while True:
        # Read the length of the image as a 32-bit unsigned int. If the
        # length is zero, quit the loop
        print('to unpack')
        image_len = struct.unpack(
            '<L', connection.read(struct.calcsize('<L')))[0]
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
        # Translate image to gray
        gray = cv2.cvtColor(opencvImage, cv2.COLOR_BGR2GRAY)
        #blur = cv2.GaussianBlur(gray,(3,3),0)
        #_,otsu = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        cv2.imshow("gray", gray)

        # Detecting april tags in the image and drawing the bounding box and center of the tag on the image

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
            cv2.line(opencvImage, ptA, ptB, (0, 255, 0), 2)
            cv2.line(opencvImage, ptB, ptC, (0, 255, 0), 2)
            cv2.line(opencvImage, ptC, ptD, (0, 255, 0), 2)
            cv2.line(opencvImage, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(opencvImage, (cX, cY), 5, (0, 0, 255), -1)
            # extract camera parameters
            fx, fy, cx, cy = camera_params
            # find camera matrix 
            K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
            # extract rotation and translation vectors
            #rvec, _ = cv2.Rodrigues(r.pose_R)
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
            
            april_distance = tag_size * fx / \
                numpy.linalg.norm(ipoints[0] - ipoints[1])
            distance = round(april_distance, 2)
            #cv2.putText(opencvImage, f'dist:{distance}', (ptA[0], ptA[1] - 30),
             #           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # find the center of the tag
            center = numpy.round(r.center).astype(int)
            center = tuple(center.ravel())
            # draw the axis of the tag
            cv2.line(opencvImage, center, tuple(ipoints[0].ravel()), (0, 0, 255), 2)
            cv2.line(opencvImage, center, tuple(ipoints[1].ravel()), (0, 255, 0), 2)
            cv2.line(opencvImage, center, tuple(ipoints[2].ravel()), (255, 0, 0), 2)
            # draw the tag family on the image
            id_fam = str(r.tag_id)
            #cv2.putText(opencvImage, id_fam, (ptA[0], ptA[1] - 15),
            #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # print the information about the tag: id, rotation, translation, center, corners, distance
            #print("[INFO] tag id: {}".format(id_fam))
            #print("Rotation: {}".format(r.pose_R))
            print("Translation: {}".format(r.pose_t))
            #cv2.putText(opencvImage, f"{numpy.round(r.pose_R[0],2)}\n {numpy.round(r.pose_R[1],2)}\n {numpy.round(r.pose_R[2],2)}", (ptA[0]+25, ptA[1] - 50),
                        # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(opencvImage, f"{numpy.round(r.pose_t[0],2)}\n {numpy.round(r.pose_t[1],2)}\n {numpy.round(r.pose_t[2],2)}", (ptA[0]+25, ptA[1] - 45),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print(r.pose_t)
            #print("Center: {}".format(r.center))
            #print("Corners: {}".format(r.corners))
            #print("Distance: {}".format(distance))
            #print("")
            # store rotation matrix and translation vector, translate rotation matrix to quaternion, make string and send it to the client
            # and send it to the client
            #rotation = r.pose_R
            #rot_quat = tf.transformations.quaternion_from_matrix(rotation)
            rotation = r.pose_R
            rotation = numpy.round(rotation, 3)
            rot_quat = transforms3d.quaternions.mat2quat(rotation)
            translation = r.pose_t
            translation = numpy.round(translation, 3)
            translation = translation.ravel()
            string = str(id_fam) + ' ' + str(rot_quat[0]) + ' ' + str(rot_quat[1]) + ' ' + str(rot_quat[2]) + ' ' + str(rot_quat[3]) + ' ' + str(translation[0]) + ' ' + str(translation[1]) + ' ' + str(translation[2])
            # print(string)
            # print("")
            #cv2.putText(opencvImage, f"quat:{numpy.round(rot_quat,2)}", (ptA[0]+25, ptA[1] - 50),
            #                cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)
            send_string = string.encode('utf-8')
            sock.sendto(send_string, (UDP_IP, UDP_PORT))
            
        cv2.imshow('Image', opencvImage)
        #cv2.imwrite(f'.\calibration\{i}.png',opencvImage)
        #out.write(opencvImage)
        image.verify()
#        print('Image is verified')
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        i += 1
finally:
    connection.close()
    print('Closing')
    server_socket.close()
