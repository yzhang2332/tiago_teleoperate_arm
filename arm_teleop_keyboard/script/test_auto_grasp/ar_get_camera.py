
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import tf_conversions
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

def rotationVectorToEulerAngles(rvec):
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

class ArucoDetector:
    def __init__(self):
        self.node_name = "aruco_detector"
        rospy.init_node(self.node_name, anonymous=True)
        self.bridge = CvBridge()
        # Change the topic to match the camera topic from your robot
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
        self.cameraMatrix = np.array([[504.33469, 0.0, 317.57],
                                      [0.0, 505.35, 234.248],
                                      [0.0, 0.0, 1.0]], dtype=float)
        self.cameraMatrix = np.array([[522.19253988, 0.0, 320],
                                      [0.0, 522.19253988, 240],
                                      [0.0, 0.0, 1.0]], dtype=float)
        
        self.distCoeffs = np.array([[-0.05378933,  0.10715212, -0.00133245, -0.00592152, -0.10959971]], dtype=float)
        self.distCoeffs = np.array([[0,  0.10715212, 0, 0, 0]], dtype=float)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.parameters = cv2.aruco.DetectorParameters()

        self.aruco_pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=1)

        rospy.on_shutdown(self.shutdown_hook)

    def detect_markers(self, frame, aruco_dict, parameters, cameraMatrix, distCoeffs):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.03, cameraMatrix, distCoeffs)

            for i, corner in enumerate(corners):
                aruco.drawDetectedMarkers(frame, corners, ids)
                euler_angles = rotationVectorToEulerAngles(rvecs[i])
                print(f"Marker ID: {ids[i][0]}")
                
                pose_msg = PoseStamped()
                # pose_msg.header.frame_id = "camera_link"  # Change this to your camera frame
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = f"aruco_marker_{ids[i][0]}"
                pose_msg.pose.position.x = tvecs[i][0][0]
                pose_msg.pose.position.y = tvecs[i][0][1]
                pose_msg.pose.position.z = tvecs[i][0][2]

                # Convert Euler angles to a quaternion
                quaternion = tf_conversions.transformations.quaternion_from_euler(math.radians(euler_angles[0]), math.radians(euler_angles[1]), math.radians(euler_angles[2]))
                pose_msg.pose.orientation = Quaternion(*quaternion)

                self.aruco_pub.publish(pose_msg)
                print("Published aruco pose")

                cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03)
        return frame

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        frame_marked = self.detect_markers(cv_image, self.aruco_dict, self.parameters, self.cameraMatrix, self.distCoeffs)
        cv2.imshow('Frame with ArUco Markers', frame_marked)
        cv2.waitKey(3)

    def shutdown_hook(self):
        rospy.loginfo(f"Shutting down {self.node_name}")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    aruco_detector = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


'''
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState

def rotationVectorToEulerAngles(rvec):
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



class ArucoDetector:
    def __init__(self):
        self.node_name = "aruco_detector"
        rospy.init_node(self.node_name, anonymous=True)
        self.bridge = CvBridge()
        # Change the topic to match the camera topic from your robot
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
        self.cameraMatrix = np.array([[504.33469, 0.0, 317.57],
                                       [0.0, 505.35, 234.248],
                                       [0.0, 0.0, 1.0]], dtype=float)
        
        # self.cameraMatrix = np.array([[509.41, 0, 320],
        #                               [0, 509.31, 240],
        #                               [0, 0, 1]], dtype=float)
        
        # self.distCoeffs = np.zeros((4, 1))  # Assuming no lens distortion
        self.distCoeffs = np.array([[-0.05378933,  0.10715212, -0.00133245, -0.00592152, -0.10959971]], dtype=float)
        # self.distCoeffs = np.array([[-0.00515119,  0.11788045,  0.00283331,  0.01335298, -0.62404533]], dtype=float)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.parameters = cv2.aruco.DetectorParameters()

        self.aruco_pub = rospy.Publisher("/aruco_pose", JointState, queue_size=1)
        self.aruco_published = False

        rospy.on_shutdown(self.shutdown_hook)


    def detect_markers(self, frame, aruco_dict, parameters, cameraMatrix, distCoeffs):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.03, cameraMatrix, distCoeffs)

            for i, corner in enumerate(corners):
                aruco.drawDetectedMarkers(frame, corners, ids)
                euler_angles = rotationVectorToEulerAngles(rvecs[i])
                print(f"Marker ID: {ids[i][0]}")
                marker_id = ids[i][0]
                X_avg = int((corners[i][0][0][0]+corners[i][0][1][0])/2)
                Y_avg = int((corners[i][0][0][1]+corners[i][0][2][1])/2)
                print(f"Location (x, y): ({X_avg}, {Y_avg})")
                #print(f"Marker ID: {ids[i][0]}, Euler Angles (Yaw): {euler_angles[2]:.2f}")
                x = tvecs[i][0][0]
                y = tvecs[i][0][1]
                z = tvecs[i][0][2]
                roll = euler_angles[0]
                pitch = euler_angles[1]
                yaw = euler_angles[2]
                print(f"Location (x, y, z): ({tvecs[i][0][0]:.4f}, {tvecs[i][0][1]:.4f}, {tvecs[i][0][2]:.4f})")
                print(X_avg, Y_avg)
                print(f"Euler Angles (Roll, Pitch, Yaw): {euler_angles[0]:.4f}, {euler_angles[1]:.4f}, {euler_angles[2]:.4f}")

                if not self.aruco_published:
                    # publish aruco pose
                    pose_msg = JointState()
                    pose_msg.name = [str(marker_id)]
                    pose_msg.position = [x, y, z, roll, pitch, yaw]
                    self.aruco_pub.publish(pose_msg)
                    print("Published aruco pose")
                    # self.aruco_published = True

                cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03)
        return frame

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        frame_marked = self.detect_markers(cv_image, self.aruco_dict, self.parameters, self.cameraMatrix, self.distCoeffs)
        cv2.imshow('Frame with ArUco Markers', frame_marked)
        cv2.waitKey(3)

    def shutdown_hook(self):
        rospy.loginfo(f"Shutting down {self.node_name}")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    aruco_detector = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
'''