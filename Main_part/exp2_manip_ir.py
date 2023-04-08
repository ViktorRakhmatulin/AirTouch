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

def coordinate_systems_transform(xee_rec, x_ee):
    '''This function calculates coordinates of the end-effector in camera coordinate system.
    Since our impaler is on the 3rd link of UR10 robot, we extract only parameters for 3 joints and calculate transform matrices for 
    end-effector in base frame, transform matrix from base to camera frame
    (since we want to avoid computing inverse matrix from camera to base frame)
    and  then end-effector coordinates in camera frame.
    Input: angles (recieved from mp.Pipe from manipulator process), connection variable from mp.Pipe()
    Output: end-effector coordinates in camera frame. 
    Note: since we are using multiprocessing, we are sending the coordinates with x_ee.send() command. 
    
    '''
    while True:
        end_effector = xee_rec.get()
        if np.array([end_effector]).any():

            #Define the extrinsic parameters of the camera
            cam_pos = np.array([0.0, 0.4, 0.54]) # Camera position
            cam_rot = np.array([-np.pi/2, 0, -3*np.pi/4]) # Camera rotation

            Tcb_hand = np.array([[np.cos(cam_rot[2]), np.sin(cam_rot[2]), 0, -cam_pos[0]*np.cos(cam_rot[2]) - cam_pos[1]*np.sin(cam_rot[2])],
                        [-np.sin(cam_rot[2])*np.cos(cam_rot[0]),np.cos(cam_rot[0])*np.cos(cam_rot[2]),np.sin(cam_rot[0]),cam_pos[0]*np.sin(cam_rot[2])*np.cos(cam_rot[0])-cam_pos[1]* np.cos(cam_rot[0])*np.cos(cam_rot[2])-cam_pos[2]*np.sin(cam_rot[0])],
                        [np.sin(cam_rot[0])* np.sin(cam_rot[2]), -np.sin(cam_rot[0])* np.cos(cam_rot[2]),np.cos(cam_rot[0]),-cam_pos[0]*np.sin(cam_rot[0])* np.sin(cam_rot[2])+cam_pos[1]*np.sin(cam_rot[0])*np.cos(cam_rot[2])-cam_pos[2]*np.cos(cam_rot[0])],
                        [0,0,0,1]])

            X_cb = Tcb_hand @ end_effector[:2]
            
            x_ee.send(X_cb)
            print(X_cb)


def image_process(x_ee):
    '''
    Input: end effector coordinates.
    Output: xD
    
    '''
    print('Image processing process started')
    fl = 0
    arduino = serial.Serial('COM7',baudrate=9600)
    time.sleep(5)
    arduino.write(b'init')

    camera_params = (506.19083684, 508.36108854,
                 317.93111342, 243.12403806)
    tag_size = 0.0375

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
    
    
    pose_prev = np.array([0,0,0])
    vel = 0
    data_file = open('./exp1/distance_vel_time1.txt','w')
    with open('./exp1/distance_vel_time1.txt','w') as data_file:
        try:
            general_start = time.time()
            while True:
                start = time.time()
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

                    if x_ee.poll():
                        x_end = x_ee.recv()
                        
                        marker_pose = np.array(r.pose_t).ravel()
                        vel = np.linalg.norm((marker_pose - pose_prev)/dt)
                        pose_prev = marker_pose
                        distance = np.linalg.norm(marker_pose - x_end)
                        message = str(distance) + ' ' + str(vel) + ' ' + str(round(time.time()-general_start,2)) + '\n'
                        data_file.write(str(message))
                        if distance < 0.2:
                            if not fl:
                                arduino.write(b'suck')
                                print('Sucking')
                                fl = 1
                        else:
                            if fl:
                                arduino.write(b'hold')
                                print('Stopping')
                                fl=0
                    
                else:
                    message = str(0) + ' ' + str(0) + ' ' + str(round(time.time()-general_start,2)) + '\n'
                    data_file.write(message)
                dt = time.time() - start


        finally:
            data_file.close()
            print('Closing')
            x_ee.close()


def manip_control_non_stop(waypoints,xee_send):
    '''
    This function is used for controlling the UR10 robot movement. Here we just move it along precalculated trajectory.
    Since we have our manipulator connection and control here, this function is also sending joint angles for further 
    impaler position in camera frame (function coordinate_systems_transform()).

    Input: trajectory (waypoints), angles_send (mp.Pipe() variable used for sending angles).
    Output: manipulator control, joint angles array.
    
    '''
    rob = urx.Robot('192.168.88.139')
    
    print('Robot connected')
    print("manip_control process started")
    
    i = 0
    while True:
        
        print(waypoints[i%len(waypoints)])
        rob.movel(waypoints[i % len(waypoints)],0.01,0.01,wait=False)
        while True:
            current_position = np.array(rob.getl()[:2])
            xee_send.put(current_position)
            time.sleep(0.1)
            if not rob.is_program_running():
                break
        i+=1

        

def main():
    try:
        t1 = [0.68325,-0.39991,0.49779,0.88,-4.36,1.04]
        t2 = [0.77786,-0.32756,0.43346,0.853,-4.298,1.135]
        t3 = [0.85331,-0.24825,0.43377,0.5521,-4.1533,1.4024]
        t4 = [0.90372,-0.33657,0.50376,1.0226,-4.3088,1.0112]
        t5 = [0.95154,-0.33621,0.49041,1.2527,-4.2915,1.1339]
        t6 = [0.82262,-0.37907,0.44094,1.3752,-4.3259,0.9087]

        waypoints = [t1,t2,t3,t4,t5,t6]
        xee_send, xee_rec = mp.Pipe()
        # angles_send, angles_rec = mp.Pipe()
        ee_coord = mp.Queue()

        manip_proc = mp.Process(target=manip_control_non_stop,args=(waypoints, ee_coord,),daemon=True) # add angles_send here so this works
        # coord_proc = mp.Process(target = coordinate_systems_transform, args = (ee_coord, xee_send,),daemon=True)
        # im_proc = mp.Process(target=image_process,args=(xee_rec,),daemon=True)

        manip_proc.start()
        # coord_proc.start()
        # im_proc.start()

        while True:
            if keyboard.is_pressed('q'):
                manip_proc.terminate()
                # coord_proc.terminate()
                # im_proc.terminate()

                break
    finally:
        print('FINALLY!')

if __name__ == '__main__':
    main()