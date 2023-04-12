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
from scipy.spatial.transform import Rotation
import pickle

folder_path = './Main_part/data/exp2/12thApril/'
filename = "Ali_no_imp" + '.txt'

file_name = folder_path + filename
def coordinate_systems_transform(ee_coord, x_ee):
    '''This function calculates coordinates of the end-effector in camera coordinate system.
    Since our impaler is on the 3rd link of UR10 robot, we extract only parameters for 3 joints and calculate transform matrices for 
    end-effector in base frame, transform matrix from base to camera frame
    (since we want to avoid computing inverse matrix from camera to base frame)
    and  then end-effector coordinates in camera frame.
    Input: angles (recieved from mp.Pipe from manipulator process), connection variable from mp.Pipe()
    Output: end-effector coordinates in camera frame. 
    Note: since we are using multiprocessing, we are sending the coordinates with x_ee.send() command. 
    q
    '''
    with open('camera_to_base.pickle','rb') as file:
        camera_to_base = pickle.load(file)
    while True:
        end_effector = ee_coord.get()
        if np.array([end_effector]).any():

            #Define the extrinsic parameters of the camera
            cam_pos = np.array([0.175, 0.445, 0.35]) # Camera position
            cam_rot = np.array([-np.pi/2, 0, -3*np.pi/4]) # Camera rotation

            Tcb_hand = np.array([[np.cos(cam_rot[2]), np.sin(cam_rot[2]), 0, -cam_pos[0]*np.cos(cam_rot[2]) - cam_pos[1]*np.sin(cam_rot[2])],
                        [-np.sin(cam_rot[2])*np.cos(cam_rot[0]),np.cos(cam_rot[0])*np.cos(cam_rot[2]),np.sin(cam_rot[0]),cam_pos[0]*np.sin(cam_rot[2])*np.cos(cam_rot[0])-cam_pos[1]* np.cos(cam_rot[0])*np.cos(cam_rot[2])-cam_pos[2]*np.sin(cam_rot[0])],
                        [np.sin(cam_rot[0])* np.sin(cam_rot[2]), -np.sin(cam_rot[0])* np.cos(cam_rot[2]),np.cos(cam_rot[0]),-cam_pos[0]*np.sin(cam_rot[0])* np.sin(cam_rot[2])+cam_pos[1]*np.sin(cam_rot[0])*np.cos(cam_rot[2])-cam_pos[2]*np.cos(cam_rot[0])],
                        [0,0,0,1]])

            T = np.zeros((4,4))
            T[:3,:3] = Rotation.from_rotvec(end_effector[3:]).as_matrix()
            T[:3,3] = end_effector[:3]
            T[3,3] = 1
            traverse = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-0.15],[0,0,0,1]])
            res = T @ traverse
            
            X_cb = camera_to_base @ res[:,3]
            # print(X_cb)
            x_ee.send(X_cb)


def image_process(x_ee):
    '''
    Input: end effector coordinates.
    Output: xD
    
    '''
    print('Image processing process started')
    fl = 0
    dt = 1
    arduino = serial.Serial('COM7',baudrate=115200)
    time.sleep(1)
    arduino.write(b'init')
    time.sleep(2)
    arduino.write(b'go to 023')
    print('Initialized')

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
    out = cv2.VideoWriter('recognition.mp4',fourcc,30,(640,480))
    pose_prev = np.array([0,0,0])
    vel = 0
    data_file = open(file_name,'w+')
    record_true = False
    try:
        ardu_time = 0
        general_start = time.time()
        #ardu_time = time.time()
        while True:
            if keyboard.is_pressed('c'):
                record_true = True
                print('recording started')

            if x_ee.poll():
                x_end = x_ee.recv()
                x_end  = x_end[:3]
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
                # x_end = x_ee.get()
                # if np.array([x_end]).any():
                #     x_end = x_end[:3]
                    
                marker_pose = np.array(r.pose_t).ravel()
                vel = np.linalg.norm((marker_pose - pose_prev)/dt)
                pose_prev = marker_pose
                distance = np.linalg.norm(marker_pose - x_end)
                message = str(round(distance,3)) + ' ' + str(round(vel,3)) + ' ' + str(round(time.time()-general_start,2)) + '\n'
                if record_true:
                    data_file.write(str(message))
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
                cv2.putText(opencvImage,f'Distance: {distance:.2f}',(ptA[0]+40, ptA[1] + 45),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                if distance < 0.35:
                    ardu_time = time.time()
                    if not fl:
                        arduino.write(b'suck 1300')
                        print('Sucking')
                        fl = 1
                        #ardu_fl.send(fl)
                        #ardu_time = time.time()
                else:
                    if fl and (time.time()-ardu_time > 0.1):
                        ardu_time = time.time()
                        arduino.write(b'hold')
                        print('Stopping')
                        fl=0
                        #ardu_fl.send(fl)
                        #ardu_time = time.tqime()
            # if time.time()-ardu_time > 4:
            #     ardu_time = time.time()
            #     arduino.write(b'hold')
            #     #print('Stopping')
            #     fl=0

                #
                
            else:
                message = str(0) + ' ' + str(0) + ' ' + str(round(time.time()-general_start,2)) + '\n'
                if record_true:
                    data_file.write(message)
                if time.time()-ardu_time > 0.1 and fl:
                    ardu_time = time.time()
                    arduino.write(b'hold')
                    print('Stopping')
                    fl=0
            out.write(opencvImage)
                # if time.time()-ardu_time > 4:
                #     ardu_time = time.time()
                #     arduino.write(b'hold')
                #     #print('Stopping')
                #     fl=0
            #ardu_fl.send(fl)
            cv2.imshow('Image',opencvImage)
            cv2.waitKey(1)

        
    finally:
        data_file.close()
        print('Closing')
        x_ee.close()
        #ardu_fl.close()


def arduino_control(ardu_fl):
    arduino = serial.Serial('COM5',baudrate=115200)
    time.sleep(1)
    arduino.write(b'init')
    time.sleep(1)
    arduino.write(b'go to 23')
    try:
        while True:
            if ardu_fl.poll():
                time.sleep(1)
                fl = ardu_fl.recv()
                if fl == 0:
                    pass
                    arduino.write(b'hold')
                else:
                    pass
                    arduino.write(b'suck 1200')
                print(fl)
    finally:
        ardu_fl.close()



def manip_control_non_stop(waypoints,ee_coord):
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
        
        current_position = np.array(rob.getl())
        ee_coord.put(current_position)
        time.sleep(0.1)

        print(waypoints[i%len(waypoints)])
        print()
        rob.movel(waypoints[i % len(waypoints)],0.018,0.05,wait=False)
        while True:
            current_position = np.array(rob.getl())
            ee_coord.put(current_position)
            time.sleep(0.1)
            if not rob.is_program_running():
                break
        i+=1

        

def main():
    try:
        t1 = [0.778,-0.305,0.440,1.1635,-4.2655,1.3380] # center far away
        t2 = [0.852,-0.218,0.414,0.164,-3.957,2.137] # bottom close
        t3 = [0.85331,-0.24825,0.43377,0.5521,-4.1533,1.4024] #up close
        t4 = [0.90372,-0.33657,0.50376,1.0226,-4.3088,1.0112] # center socket
        t5 = [0.95154,-0.33621,0.49041,1.2527,-4.2915,1.1339] #socket
        t6 = [0.82262,-0.37907,0.44094,1.3752,-4.3259,0.9087] #far away
        t7 = [0.84832,-0.27680,0.35760,0.8533,-4.1342,1.9250] # center down
        t8 = [0.87760, -0.20860, 0.35763, 0.0348, 1.3320, -0.9303] #toxic position (bottom CLOSE!!!!)

        waypoints = [t8,t3,t4,t6,t7,t4,t5,t4]
        # dir={"0": , "2": 2, "t": 3, "t3": 4, "t3": 5, "t3": 1, "t3": 1 }
        xee = mp.Queue()
        # angles_send, angles_rec = mp.Pipe()
        ee_coord = mp.Queue()
        ee_coord_send, ee_coord_rec = mp.Pipe()
        #ardu_send, ardu_rec = mp.Pipe()

        manip_proc = mp.Process(target=manip_control_non_stop,args=(waypoints, ee_coord,),daemon=True) # add angles_send here so this works
        coord_proc = mp.Process(target = coordinate_systems_transform, args = (ee_coord, ee_coord_send,),daemon=True)
        im_proc = mp.Process(target=image_process,args=(ee_coord_rec,),daemon=True)
        #ardu_proc = mp.Process(target=arduino_control,args=(ardu_rec,),daemon=True)

        manip_proc.start()
        coord_proc.start()
        im_proc.start()
        #ardu_proc.start()

        while True:
            if keyboard.is_pressed('q'):
                manip_proc.terminate()
                coord_proc.terminate()
                im_proc.terminate()
                #ardu_proc.terminate()

                break
    finally:
        print('FINALLY!')

if __name__ == '__main__':
    main()