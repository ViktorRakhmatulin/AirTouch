import cv2
import numpy as np

# Load the chessboard pattern
pattern_size = (6, 9)
square_size = 0.1  # size of each square in meters

# Calculate the world coordinates of the chessboard corners
pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2) * square_size

# Set up the object points and image points for the chessboard pattern
obj_points = []
img_points = []

# Capture images of the chessboard pattern from the camera
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Find the chessboard corners in the image
    found, corners = cv2.findChessboardCorners(gray, pattern_size)
    if found:
        # Add the object points and image points to the lists
        obj_points.append(pattern_points)
        img_points.append(corners)
        
        # Draw the corners on the image
        cv2.drawChessboardCorners(frame, pattern_size, corners, found)
    
    # Display the image
    cv2.imshow("Frame", frame)
    
    # Check if the user pressed the 'q' key to stop capturing images
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture
cap.release()
cv2.destroyAllWindows()

# Use the object points and image points to calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

# Save the calibration parameters to a file
np.savez("calibration.npz", mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)