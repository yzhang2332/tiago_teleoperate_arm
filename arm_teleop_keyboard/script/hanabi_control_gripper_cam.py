import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import JointState
import numpy as np


def image_callback(msg, aruco_pub):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to grayscale (required for ArUco detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Define the dictionary and parameters for ArUco marker detection
        aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)  # You can choose different dictionaries
        parameters = aruco.DetectorParameters()

        # Detect ArUco markers in the image
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # If markers are detected, draw them
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            for i, corner in enumerate(corners):
                # print(f"Marker ID: {ids[i][0]}")
                marker_id = ids[i][0]
                X_avg = int((corners[i][0][0][0]+corners[i][0][1][0])/2)
                Y_avg = int((corners[i][0][0][1]+corners[i][0][1][1])/2)


                marker_corners = corner[0]  # Extract the corner points from the list
                # Calculate the average X and Y for this marker
                avg_x = np.mean(marker_corners[:, 0])  # Average X-coordinate
                avg_y = np.mean(marker_corners[:, 1])  # Average Y-coordinate


                # publish aruco pose
                pose_msg = JointState()
                pose_msg.name = [str(marker_id)]
                pose_msg.position = [avg_x, avg_y]
                aruco_pub.publish(pose_msg)



        # Display the image
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr("Failed to convert image: %s", str(e))

def main():
    rospy.init_node('camera_subscriber', anonymous=True)
    aruco_pub = rospy.Publisher("/aruco_pose_robot_camera", JointState, queue_size=1)

    rospy.Subscriber('/camera/image_raw', Image, image_callback, aruco_pub)
    rospy.spin()

if __name__ == '__main__':
    main()