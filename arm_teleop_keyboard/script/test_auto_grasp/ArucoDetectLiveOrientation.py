import cv2
import cv2.aruco as aruco
import numpy as np
import math

def rotationVectorToEulerAngles(rvec):
    # Calculates rotation matrix to euler angles
    R, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.degrees(x), np.degrees(y), np.degrees(z)

def detect_markers(frame, aruco_dict, parameters, cameraMatrix, distCoeffs):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect markers
    corners, ids, rejectedCandidates = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    rvecs, tvecs, _objPoints = None, None, None
    if ids is not None:
        # Estimate pose of each marker
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.03, cameraMatrix, distCoeffs)
        # # Draw markers and axes
        # for rvec, tvec in zip(rvecs, tvecs):
        #     aruco.drawDetectedMarkers(frame, corners, ids)
        #     euler_angles = rotationVectorToEulerAngles(rvec)
        #     print("Euler Angles (Roll, Pitch, Yaw):", euler_angles)

        for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            aruco.drawDetectedMarkers(frame, corners, ids)
            euler_angles = rotationVectorToEulerAngles(rvec)
            print(f"Marker ID: {ids[i][0]}")
            # cv2.circle(
            # frame,
            # (int(corners[i][0][0][0]), int(corners[i][0][0][1])),
            # radius=40,
            # color=(0, 0, 255),
            # thickness=15,
            # )
            X_avg = int((corners[i][0][0][0]+corners[i][0][1][0])/2)
            Y_avg = int((corners[i][0][0][1]+corners[i][0][2][1])/2)
            # cv2.circle(
            # frame,
            # (X_avg, Y_avg),
            # radius=40,
            # color=(0, 0, 255),
            # thickness=15,
            # )
            #print(f"Location (x, y): ({X_avg}, {Y_avg})")
            print(f"Location (x, y, z): ({tvec[0][0]:.2f}, {tvec[0][1]:.2f}, {tvec[0][2]:.2f})")
            #print(f"Euler Angles (Roll, Pitch, Yaw): ({euler_angles[0]:.2f}, {euler_angles[1]:.2f}, {euler_angles[2]:.2f})")
            print(f"Euler Angles (Yaw): ({euler_angles[2]:.2f})")

            cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.01)  # Draw a 3cm axis
    return frame, ids, rvecs, tvecs

# Initialize video capture
cap = cv2.VideoCapture(0)

# Placeholder values for camera matrix and distortion coefficients
cameraMatrix = np.array([[1416.79, 0, 975.76],
                         [0, 1421.52, 536.014],
                         [0, 0, 1]], dtype=float)
#distCoeffs = np.zeros((4, 1))  # Assuming no lens distortion
distCoeffs = np.array([[-1.68764996e-03,  8.50329380e-01,  1.19705902e-02,  2.62272365e-03,
  -1.81031839e+00]], dtype=float)

# cameraMatrix = np.array([[132.90925393, 0, 959.50922447],
#                          [0, 321.20529879, 539.50654642],
#                          [0, 0, 1]], dtype=float)
# #distCoeffs = np.zeros((4, 1))  # Assuming no lens distortion
# distCoeffs = np.array([[ -2.25747298e-02,  1.09554381e-04, -1.27423019e-03, -4.71055449e-03, -1.43673521e-07]], dtype=float)

# Define the ArUco dictionary and detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame. Exiting...")
            break
        
        # Detect ArUco markers and estimate their pose
        frame_marked, ids, rvecs, tvecs = detect_markers(frame, aruco_dict, parameters, cameraMatrix, distCoeffs)
        
        # Display the frame with markers and pose estimations
        cv2.imshow('Frame', frame_marked)

        # Break the loop with 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
