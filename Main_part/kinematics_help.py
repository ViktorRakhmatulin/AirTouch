import numpy as np
import sympy as sp
import roboticstoolbox as robt
import matplotlib.pyplot as plt
import scipy
import math
import spatialmath
import urx

# Define the Denavit-Hartenberg parameters of the robot arm
a = np.array([0, -0.612, -0.5723/2])     # Link lengths
alpha = np.array([np.pi/2, 0, 0]) # Twist angles
d = np.array([0.1273, 0, 0])     # Link offsets
theta = np.array([0.5, -0.2, 0.3]) # Joint angles

#Define the extrinsic parameters of the camera
cam_pos = np.array([0.1, 0.1, 0.1]) # Camera position
cam_rot = np.array([np.pi/2, 0, np.pi/4]) # Camera rotation

# Define function for transform matrix
def transform_matrix(theta, a, d, alpha):
    '''Function for transform matrix'''
    T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                  [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                  [0, np.sin(alpha), np.cos(alpha), d],
                  [0, 0, 0, 1]])
    return T

T01 = transform_matrix(theta[0],a[0],d[0],alpha[0])

T12 = transform_matrix(theta[1],a[1],d[1],alpha[1])

T23 = transform_matrix(theta[2],a[2],d[2],alpha[2])

T03 = np.dot(T01, T12)
T03 = np.dot(T03, T23)

#Coordinates of the end effector:
X_ee = np.array([T03[0,3],T03[1,3], T03[2,3]])

Tcb_hand = np.array([[np.sqrt(2)/2, np.sqrt(2)/2, 0, -0.14142136],
                     [0,0,1,-0.1],
                     [np.sqrt(2)/2, -np.sqrt(2)/2,0,0],
                     [0,0,0,1]])

X_cb = np.dot(Tcb_hand, T03)
print(X_cb)