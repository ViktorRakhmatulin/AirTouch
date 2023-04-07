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

def transform_matrix(theta, a, d, alpha):
    '''Function for transform matrix for all manipulator related stuff'''
    T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                  [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                  [0, np.sin(alpha), np.cos(alpha), d],
                  [0, 0, 0, 1]])
    return T

def camera_base_transform_matrix(trans_coord, cam_rot, order):
    '''
    This function calculates transform matrix for 3 rotations (1 for each axis), and then calculates inverse matrix (we must use it in our case. 
    Later: create a class for doing inverse matrix.)

    In our function calculates transform matrix from base to camera, inverses it, and we have transform matrix from camera to base. We can further use it
    To calculate end-effector coordinates.

    Input: transfer coordinates (for each axis, 3x1 size), rotation angles (theta_x, theta_y, theta_z), order in which matrices are rotating.

    Write later))) 
    '''
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
    while True:
        angles = angles_rec.get()
        if np.array([angles]).any():
            theta = np.array([angles[0], angles[1], angles[2]])
            a = np.array([0, -0.612, -0.5723/2])     # Link lengths
            alpha = np.array([np.pi/2, 0, 0]) # Twist angles
            d = np.array([0.1273, 0, 0])     # Link offsets

            #Define the extrinsic parameters of the camera
            cam_pos = np.array([0.0, 0.4, 0.54]) # Camera position
            cam_rot = np.array([-np.pi/2, 0, -3*np.pi/4]) # Camera rotation

            #Calculate transform matrix for each joint
            T01 = transform_matrix(theta[0],a[0],d[0],alpha[0])

            T12 = transform_matrix(theta[1],a[1],d[1],alpha[1])

            T23 = transform_matrix(theta[2],a[2],d[2],alpha[2])

            #Calculate transform from end-effector to base frame coordinate system
            T03 = T01 @ T12 @ T23

            Tcb_hand = np.array([[np.cos(cam_rot[2]), np.sin(cam_rot[2]), 0, -cam_pos[0]*np.cos(cam_rot[2]) - cam_pos[1]*np.sin(cam_rot[2])],
                        [-np.sin(cam_rot[2])*np.cos(cam_rot[0]),np.cos(cam_rot[0])*np.cos(cam_rot[2]),np.sin(cam_rot[0]),cam_pos[0]*np.sin(cam_rot[2])*np.cos(cam_rot[0])-cam_pos[1]* np.cos(cam_rot[0])*np.cos(cam_rot[2])-cam_pos[2]*np.sin(cam_rot[0])],
                        [np.sin(cam_rot[0])* np.sin(cam_rot[2]), -np.sin(cam_rot[0])* np.cos(cam_rot[2]),np.cos(cam_rot[0]),-cam_pos[0]*np.sin(cam_rot[0])* np.sin(cam_rot[2])+cam_pos[1]*np.sin(cam_rot[0])*np.cos(cam_rot[2])-cam_pos[2]*np.cos(cam_rot[0])],
                        [0,0,0,1]])

            X_cb = np.dot(Tcb_hand, T03)

            X_cb = np.array([X_cb[0,3], X_cb[1,3], X_cb[2,3]])
            
            x_ee.send(X_cb)
            #print(X_cb)

def arduino_theta_control(coord_variable):
    print('arduino process started')

    #arduino = serial.Serial('COM6',9600)
    
    while True:
        

        #arduino_state = 0
        
        if coord_variable.poll():
            coord = coord_variable.recv()
            x_end = coord[0]
            marker = coord[1]
            if not np.array([marker]).any():
                #print(x_end)
                pass
            else:
                marker = marker.ravel()
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
                distance = np.linalg.norm(marker - x_end)
                
                print(f'Я выпил {theta:03} бутылок пива по цене {distance:.3f}')
                
                if distance < 0.2:
                    pass
                    #arduino.write('that turns me on')
                    print('that turns me on')



def image_process(x_ee, coord_variable):
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
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    #out = cv2.VideoWriter('measuring_distance.mp4',fourcc,30,(640,480))
    #i = 0
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
                        
                        #coord_variable.send((x_end, r.pose_t))
                    marker_pose = np.array(r.pose_t).ravel()
                    vel = np.linalg.norm((marker_pose - pose_prev)/dt)
                    pose_prev = marker_pose
                    distance = np.linalg.norm(marker_pose)
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
                    if x_ee.poll():
                        x_end = x_ee.recv()
                        coord_variable.send((x_end, None))
                    message = str(0) + ' ' + str(0) + ' ' + str(round(time.time()-general_start,2)) + '\n'
                    data_file.write(message)
                dt = time.time() - start


        finally:
            data_file.close()
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
    
    print('Robot connected')
    print("manip_control process started")
    
    i = 0
    while True:
        # current_joints = rob.getj()
        # angles = np.array([current_joints[0],current_joints[1], current_joints[2]])
        # print(i,angles)
        # angles_send.put(angles)
        # time.sleep(0.1)
        
        print(waypoints[i%len(waypoints)])
        rob.movel(waypoints[i % len(waypoints)],0.01,0.01,wait=False)
        while True:
            current_joints = rob.getj()
            current_position = rob.getl()
            angles = np.array([current_joints[0],current_joints[1], current_joints[2]])
            angles_send.put(angles)
            time.sleep(0.1)
            if not rob.is_program_running():
                break
        i+=1

        

def main():
    try:
        home = [0.692,-0.106,0.527,1.28,1.19,1.21]
        goal = [0.857,-0.099,0.492,1.28,1.19,1.21]
        t3 = [0.847,-0.147,0.485,0.9,1.37,1.34]
        t4 = [0.872,-0.0328,0.501,1.34,1.00,0.9]

        waypoints = [home,goal,t3,t4,goal]
        xee_send, xee_rec = mp.Pipe()
        # angles_send, angles_rec = mp.Pipe()
        q_angles = mp.Queue()
        coord_send, coord_rec = mp.Pipe()
        # manip_proc = mp.Process(target=manip_control_non_stop,args=(waypoints, q_angles,),daemon=True) # add angles_send here so this works
        # coord_proc = mp.Process(target = coordinate_systems_transform, args = (q_angles, xee_send,),daemon=True)
        im_proc = mp.Process(target=image_process,args=(xee_rec, coord_send,),daemon=True)
        # arduino_proc = mp.Process(target = arduino_theta_control, args = (coord_rec,),daemon=True)
        # manip_proc.start()
        # coord_proc.start()
        im_proc.start()
        # arduino_proc.start()
        while True:
            if keyboard.is_pressed('q'):
                # manip_proc.terminate()
                # coord_proc.terminate()
                im_proc.terminate()
                # arduino_proc.terminate()
                break
    finally:
        print('FINALLY!')

if __name__ == '__main__':
    main()
