#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from arm_teleop_keyboard.msg import PoseObj
import tf_conversions

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
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw/compressed", CompressedImage, self.image_callback)
        #self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
        self.cameraMatrix = np.array([[515.5234, 0.0, 318.3147],
                                       [0.0, 515.98793, 225.1747],
                                       [0.0, 0.0, 1.0]], dtype=float)
        # self.cameraMatrix = np.array([[522.19253988, 0.0, 320],
        #                               [0.0, 522.19253988, 240],
        #                               [0.0, 0.0, 1.0]], dtype=float)
        
        # self.distCoeffs = np.zeros((4, 1))  # Assuming no lens distortion
        self.distCoeffs = np.array([[0.05305353,  -0.26613098, -0.00728184, -0.00124051, 0.24547245]], dtype=float)
        # self.distCoeffs = np.array([[0,  0, 0, 0, 0]], dtype=float)


        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        # self.aruco_pub = rospy.Publisher("/aruco_pose", PoseObj, queue_size=1)
        # self.aruco_published = False

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
                
                pose_msg = PoseStamped()
                # pose_msg.header.frame_id = "camera_link"  # Change this to your camera frame
                pose_msg.header.stamp = rospy.Time.now()
                # pose_msg.header.frame_id = f"xtion_optical_frame_{ids[i][0]}"
                pose_msg.header.frame_id = "xtion_optical_frame"
                pose_msg.pose.position.x = tvecs[i][0][0]
                pose_msg.pose.position.y = tvecs[i][0][1]
                pose_msg.pose.position.z = tvecs[i][0][2]

                # Convert Euler angles to a quaternion
                quaternion = tf_conversions.transformations.quaternion_from_euler(math.radians(euler_angles[0]), math.radians(euler_angles[1]), math.radians(euler_angles[2]))
                pose_msg.pose.orientation = Quaternion(*quaternion)

                object_pose = PoseObj()
                object_pose.id = ids[i][0]
                object_pose.pose = pose_msg

                # self.aruco_pub.publish(object_pose)
                
                # print("Published aruco pose")

                cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03)
        return frame

    def image_callback(self, msg):
        try:
            #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
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