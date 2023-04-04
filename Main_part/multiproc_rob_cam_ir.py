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

def transform_matrix(theta, a, d, alpha):
    '''Function for transform matrix'''
    T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                  [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                  [0, np.sin(alpha), np.cos(alpha), d],
                  [0, 0, 0, 1]])
    return T

def coordinate_systems_transform(angles_rec, x_ee):
    '''This function calculates coordinates of the end-effector in camera coordinate system.
    Since our impaler is on the 3rd link of UR10 robot, we extract only parameters for 3 joints and calculate transform matrices for 
    end-effector in base frame, transform matrix from base to camera frame
    (since we want to avoid computing inverse matrix from camera to base frame)
    and  then end-effector coordinates in camera frame.
    Input: angles (recieved from mp.Pipe from manipulator process), connection variable from mp.Pipe()
    Output: end-effector coordinates in camera frame. 
    Note: since we are using multiprocessing, we are sending the coordinates with x_ee.send() command. 
    
    '''
    if angles_rec.recv():
        angles = angles_rec
        theta = np.array([angles[0], angles[1], angles[2]])
        a = np.array([0, -0.612, -0.5723/2])     # Link lengths
        alpha = np.array([np.pi/2, 0, 0]) # Twist angles
        d = np.array([0.1273, 0, 0])     # Link offsets

        #Define the extrinsic parameters of the camera
        cam_pos = np.array([0.1, 0.1, 0.1]) # Camera position
        cam_rot = np.array([np.pi/2, 0, np.pi/4]) # Camera rotation

        #Calculate transform matrix for each joint
        T01 = transform_matrix(theta[0],a[0],d[0],alpha[0])

        T12 = transform_matrix(theta[1],a[1],d[1],alpha[1])

        T23 = transform_matrix(theta[2],a[2],d[2],alpha[2])

        #Calculate transform from end-effector to base frame coordinate system
        T03 = T01 @ T12 @ T23

        Tcb_hand = np.array([[np.sqrt(2)/2, np.sqrt(2)/2, 0, -0.14142136],
                        [0,0,1,-0.1],
                        [np.sqrt(2)/2, -np.sqrt(2)/2,0,0],
                        [0,0,0,1]])

        X_cb = np.dot(Tcb_hand, T03)

        X_cb = np.array([X_cb[0,3], X_cb[1,3], X_cb[2,3]])
        print(f'TRANSFORM END-EFFECTOR: {X_cb}')
        
        x_ee.send(X_cb)
        #print(X_cb)

def arduino_theta_control(coord_variable):
    #arduino = serial.Serial('COM6',9600)
    while True:
        

        #arduino_state = 0
        if coord_variable.recv():
            x_end = coord_variable[0]
            marker = coord_variable[1]
            ee_to_marker = marker - x_end
            cos_alpha = np.dot(x_end, marker) / (np.linalg.norm(x_end)*np.linalg.norm(marker))
            sin_alpha = np.sqrt(1 - np.power(cos_alpha,2))
            cos_pitheta = np.dot(ee_to_marker, marker) / (np.linalg.norm(ee_to_marker)*np.linalg.norm(marker))
            cos_theta = -cos_pitheta
            sin_theta = np.linalg.norm(marker) * sin_alpha / np.linalg.norm(ee_to_marker)
            theta = np.arctan2(sin_theta, cos_theta)
            theta = np.rad2deg(theta)
            theta = int(theta)
            #arduino.write(f'go to {theta:03}')
            print(f'Я выпил {theta:03} бутылок пива')
            distance = np.linalg.norm(marker - x_end)
            if distance < 0.2:
                #arduino.write('that turns me on')
                print('that turns me on')



def image_process(x_ee, coord_variable):
    '''
    Input: end effector coordinates.
    Output: xD
    
    '''
    camera_params = (506.19083684, 508.36108854,
                 317.93111342, 243.12403806)
    tag_size = 0.0375

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
    #out = cv2.VideoWriter('measuring_distance.mp4',fourcc,30,(640,480))
    #i = 0
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
            opencvImage = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
            # Translate image to gray
            gray = cv2.cvtColor(opencvImage, cv2.COLOR_BGR2GRAY)
            #blur = cv2.GaussianBlur(gray,(3,3),0)
            #_,otsu = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            # cv2.imshow("gray", gray)

            # Detecting april tags in the image and drawing the bounding box and center of the tag on the image

            results = detector.detect(
                gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)
            #change

            '''if not results and arduino_state == 1:
                arduino_state = 0
                arduino.write(b'0') '''
    #        print("[INFO] {} total AprilTags detected".format(len(results)))
            if results:
                r = results[0]
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
                K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
                # extract rotation and translation vectors
                #rvec, _ = cv2.Rodrigues(r.pose_R)
                rvec = r.pose_R

                # print(r.pose_t)

                '''
                HERE WE SEND THE DISTANCE BETWEEEN END EFFECTOR AND MARKERS
                '''
                x_end = x_ee.recv()
                
                if x_end:
                    coord_variable.send((x_ee, r.pose_t))


                #Change it
                '''april_distance = np.linalg.norm(r.pose_t)
                if (april_distance < 0.2 and arduino_state == 0):
                    arduino_state = 1
                    arduino.write(b'1')
                    conn.send(1)
                elif (april_distance > 0.2 and arduino_state == 1):
                    arduino_state = 0
                    arduino.write(b'0')'''

                dcoeffs = np.zeros(5)
                # find object points of the tag 
                opoints = np.float32([[1, 0, 0],
                                        [0, -1, 0],
                                        [0, 0, -1]]).reshape(-1, 3) * tag_size
                # project object points to image plane
                ipoints, _ = cv2.projectPoints(opoints, rvec, r.pose_t, K, dcoeffs)
                ipoints = np.round(ipoints).astype(int)

                # find the center of the tag
                center = np.round(r.center).astype(int)
                center = tuple(center.ravel())
                # draw the axis of the tag
                cv2.line(opencvImage, center, tuple(ipoints[0].ravel()), (0, 0, 255), 2)
                cv2.line(opencvImage, center, tuple(ipoints[1].ravel()), (0, 255, 0), 2)
                cv2.line(opencvImage, center, tuple(ipoints[2].ravel()), (255, 0, 0), 2)
                # draw the tag family on the image
                id_fam = str(r.tag_id)
                

                # print("Translation: {}".format(r.pose_t))

                cv2.putText(opencvImage, f"x,y,z: {np.round(r.pose_t[0,0],3)} {np.round(r.pose_t[1,0],3)} {np.round(r.pose_t[2,0],3)}", (ptA[0]+25, ptA[1] - 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(opencvImage,f'Distance: {np.linalg.norm(r.pose_t):.3f}',(ptA[0]+40, ptA[1] - 65),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
    finally:
        print('Closing')
        x_ee.close()


def manip_control_non_stop(waypoints,angles_send):
    '''
    This function is used for controlling the UR10 robot movement. Here we just move it along precalculated trajectory.
    Since we have our manipulator connection and control here, this function is also sending joint angles for further 
    impaler position in camera frame (function coordinate_systems_transform()).

    Input: trajectory (waypoints), angles_send (mp.Pipe() variable used for sending angles).
    Output: manipulator control, joint angles array.
    
    '''
    current_joints = []
    rob = urx.Robot('192.168.88.139')
    rob.movel(waypoints[0])
    print('Robot connected')
    print("manip_control process started")
    # rob.movel(goal)
    i = 0
    while True:
        print(waypoints[i%len(waypoints)])
        rob.movel(waypoints[i % len(waypoints)],0.01,0.01,wait=False)
        while True:
            current_joints = rob.getj()
            angles = np.array([current_joints[0],current_joints[1], current_joints[2]])
            angles_send.send(angles)
            time.sleep(0.1)
            if not rob.is_program_running():
                break
        i+=1

        

def main():
    try:
        home = [0.692,-0.106,0.527,0.925,-4.326,0.986]
        goal = [0.857,-0.099,0.492,1.3268,-4.338,0.956]
        t3 = [0.847,-0.147,0.485,1.793,-4.293,0.623]
        t4 = [0.872,-0.0328,0.501,0.62,-4.2,1.5]
        waypoints = [home,goal,t3,t4,goal]
        xee_send, xee_rec = mp.Pipe()
        angles_send, angles_rec = mp.Pipe()
        coord_send, coord_rec = mp.Pipr()
        manip_proc = mp.Process(target=manip_control_non_stop,args=(waypoints, angles_send,)) # add angles_send here so this works
        coord_proc = mp.Process(target = coordinate_systems_transform, args = (angles_rec, xee_send,))
        im_proc = mp.Process(target=image_process,args=(xee_rec, coord_send,))
        arduino_proc = mp.Process(target = arduino_theta_control, args = (coord_rec,))
        manip_proc.start()
        coord_proc.start()
        im_proc.start()
        arduino_proc.start()

    finally:
        manip_proc.join()
        im_proc.join()
        coord_proc.join()
        arduino_proc.join()

if __name__ == '__main__':
    main()
