import multiprocessing as mp
import cv2
import numpy as np
import pupil_apriltags as apriltag
import urx
import struct
import collections
collections.Iterable = collections.abc.Iterable
import time
import serial
import socket
import io
from PIL import Image
import keyboard
import os
from scipy.spatial.transform import Rotation

start = time.time()

folder_path = './Main_part/data/exp1_dist/protocol_relocations/'
filename = "ttest" + '.txt'

file = open(folder_path + filename,'w+')
file.write('hand to marker: 0.110\n')
file.write('timestamp dist x y z rx ry rz\n')

print('Image processing process started')
camera_params = (506.19083684, 508.36108854,
                317.93111342, 243.12403806)
tag_size = 0.035

server_socket = socket.socket()
server_socket.bind(('0.0.0.0', 8000))
server_socket.listen(0)
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
#out = cv2.VideoWriter('measuring_distance.mp4',fourcc,30,(640,480))
#i = 0
pose_prev = np.array([0,0,0])
vel = 0
try:
    while True:
        # Read the length of the image as a 32-bit unsigned int. If the
        # length is zero, quit the loop
        image_len = struct.unpack(
            '<L', connection.read(struct.calcsize('<L')))[0]
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
        opencvImage = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        # Translate image to gray
        gray = cv2.cvtColor(opencvImage, cv2.COLOR_BGR2GRAY)

        # Detecting april tags in the image and drawing the bounding box and center of the tag on the image

        results = detector.detect(
            gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)
        #change

        if results:
            r = results[0]
            '''
            HERE WE SEND THE DISTANCE BETWEEEN END EFFECTOR AND MARKERS
            '''
            distance = np.linalg.norm(r.pose_t)
            # (ptA, ptB, ptC, ptD) = r.corners
            # ptB = (int(ptB[0]), int(ptB[1]))
            # ptC = (int(ptC[0]), int(ptC[1]))
            # ptD = (int(ptD[0]), int(ptD[1]))
            # ptA = (int(ptA[0]), int(ptA[1]))
            # # draw the bounding box of the AprilTag detection
            # cv2.line(opencvImage, ptA, ptB, (0, 255, 0), 2)
            # cv2.line(opencvImage, ptB, ptC, (0, 255, 0), 2)
            # cv2.line(opencvImage, ptC, ptD, (0, 255, 0), 2)c
            # cv2.line(opencvImage, ptD, ptA, (0, 255, 0), 2)
            # cv2.putText(opencvImage, f"x,y,z: {np.round(r.pose_t[0,0],2)} {np.round(r.pose_t[1,0],2)} {np.round(r.pose_t[2,0],2)}", (ptA[0]+25, ptA[1] - 45),
            #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # cv2.imshow('image',opencvImage)
            # cv2.waitKey(1)
            print(distance)
            rot_vec = Rotation.from_matrix(r.pose_R)
            if keyboard.is_pressed('c'):
                file.write(f'{(time.time()-start):.3f} {distance:.3f} {r.pose_t[0,0]:.4f} {r.pose_t[1,0]:.4f} {r.pose_t[2,0]:.4f} {rot_vec.as_rotvec()[0]:.4f} {rot_vec.as_rotvec()[1]:.4f} {rot_vec.as_rotvec()[2]:.4f} \n')
        if keyboard.is_pressed('q'):
            break
finally:
    print('Closing')
    file.close()
