import multiprocessing as mp
import cv2
import numpy as np
import pupil_apriltags as apriltag
import urx
import collections
collections.Iterable = collections.abc.Iterable
import time
import serial

def image_process(conn):
    camera_params = (603.12192992, 605.38959647,
                    337.64128438, 255.81680916)
    tag_size = 0.0375

    # Initializing detector
    detector = apriltag.Detector(
        families='tag36h11',
        nthreads=10,
        quad_decimate=0.5,
        quad_sigma=0,
        refine_edges=1,
        decode_sharpening=0.6,
        debug=0)
    arduino = serial.Serial('COM6',9600)

    arduino_state = 0
    
    cap = cv2.VideoCapture(0)
    try:
        while True:
            # Read the length of the image as a 32-bit unsigned int. If the
            # length is zero, quit the loop

            ret,opencvImage = cap.read()
            # Translate image to gray
            gray = cv2.cvtColor(opencvImage, cv2.COLOR_BGR2GRAY)
            #blur = cv2.GaussianBlur(gray,(3,3),0)
            #_,otsu = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            # cv2.imshow("gray", gray)

            # Detecting april tags in the image and drawing the bounding box and center of the tag on the image

            results = detector.detect(
                gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)
            
            if not results and arduino_state == 1:
                arduino_state = 0
                arduino.write(b'0') 
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
                K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
                # extract rotation and translation vectors
                #rvec, _ = cv2.Rodrigues(r.pose_R)
                rvec = r.pose_R

                # print(r.pose_t)
                
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
        conn.close()


def manip_control_non_stop(waypoints):
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
            print(current_joints)
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
        # parent,child = mp.Pipe()
        # im_proc = mp.Process(target=image_process,args=(parent,))s
        manip_proc = mp.Process(target=manip_control_non_stop,args=(waypoints,))
        # im_proc.start()
        manip_proc.start()
    finally:
        manip_proc.join()
        # im_proc.join()

if __name__ == '__main__':
    main()
