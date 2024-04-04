#!/usr/bin/env python
# import rospy
# import tf
# from geometry_msgs.msg import PointStamped

# class CameraToBaseTranslator:

#     def __init__(self):
#         rospy.init_node('camera_to_base_translator', anonymous=True)

#         self.listener = tf.TransformListener()
#         rospy.sleep(1)  # Giving some time for the listener to get ready

#     def run(self):
#         rate = rospy.Rate(10.0)  # 10 Hz
#         while not rospy.is_shutdown():
#             try:
#                 (trans, rot) = self.listener.lookupTransform('/torso_lift_link', '/xtion_link', rospy.Time(0))
#                 print("Translation: {}".format(trans))
#                 print("Rotation in Quaternion: {}".format(rot))
#             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                 continue

#             rate.sleep()

# def main():
#     translator = CameraToBaseTranslator()
#     translator.run()

# if __name__ == '__main__':
#     main()

import rospy
import tf
import tf_conversions
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

class CoordinateTranslator:

    def __init__(self):

        rospy.init_node('coordinate_translator', anonymous=True)
        self.listener = tf.TransformListener()

        self.aruco_sub = rospy.Subscriber("/aruco_pose", JointState, self.aruco_callback)

        rospy.sleep(1)  # Wait for the listener to get ready

        self.aruco_pose = []


    def publish_static_transformation(self, x, y, z, roll, pitch, yaw):

        # Create a StaticTransformBroadcaster object
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Create a TransformStamped message
        static_transform_stamped = TransformStamped()
        
        # Fill the message with the necessary data
        static_transform_stamped.header.stamp = rospy.Time.now()
        # static_transform_stamped.header.frame_id = '/xtion_rgb_optical_frame'
        static_transform_stamped.header.frame_id = '/xtion_optical_frame'
        static_transform_stamped.child_frame_id = '/aruco_marker_frame'
        static_transform_stamped.transform.translation.x = x
        static_transform_stamped.transform.translation.y = y
        static_transform_stamped.transform.translation.z = z
        
        quaternion = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
        static_transform_stamped.transform.rotation.x = quaternion[0]
        static_transform_stamped.transform.rotation.y = quaternion[1]
        static_transform_stamped.transform.rotation.z = quaternion[2]
        static_transform_stamped.transform.rotation.w = quaternion[3]

        # Send the transformation
        static_broadcaster.sendTransform(static_transform_stamped)
        rospy.loginfo("Static transform published!")
        rospy.loginfo("Sleeping for 2 seconds")
        rospy.sleep(2)


    def aruco_callback(self, msg: JointState):
        pose_vec = msg.position
        if len(self.aruco_pose) == 0:
            self.aruco_pose = pose_vec
            self.publish_static_transformation(pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5])
            # rospy.loginfo("Retrieving the transformation after 2 seconds!")
            # rospy.sleep(2)
            trans, rot, rot_euler = self.get_transformation()
            print("Point in base frame: ", trans)
            print("Orientation in base frame (roll, pitch, yaw): ", rot_euler)


    def get_transformation(self):
        #(trans, rot) = self.listener.lookupTransform("/base_footprint", "/aruco_marker_frame", rospy.Time(0))
        (trans, rot) = self.listener.lookupTransform("/torso_lift_link", "/aruco_marker_frame", rospy.Time(0))
        # print("Translation: ", trans)
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        # print("Rotation: ", rot_euler)
        # new_pose_dict = {"translation": trans, "rotation": rot}
        # return new_pose_dict
        return trans, rot, rot_euler


    # def transform_point(self, x, y, z, roll, pitch, yaw):
    #     # Wait for the listener to get the first transformation
    #     self.listener.waitForTransform('/torso_lift_link', '/xtion_rgb_optical_frame', rospy.Time(0), rospy.Duration(4.0))

    #     # Create a PoseStamped message containing the point in the camera frame
    #     point_camera_frame = PoseStamped()
    #     point_camera_frame.header.frame_id = '/xtion_rgb_optical_frame'
    #     point_camera_frame.pose.position.x = x
    #     point_camera_frame.pose.position.y = y
    #     point_camera_frame.pose.position.z = z

    #     quaternion = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
    #     point_camera_frame.pose.orientation.x = quaternion[0]
    #     point_camera_frame.pose.orientation.y = quaternion[1]
    #     point_camera_frame.pose.orientation.z = quaternion[2]
    #     point_camera_frame.pose.orientation.w = quaternion[3]

    #     # Transform the point from the camera frame to the base frame
    #     point_base_frame = self.listener.transformPose('/torso_lift_link', point_camera_frame)

    #     print("Point in base frame: ", point_base_frame.pose.position)
    #     euler = tf_conversions.transformations.euler_from_quaternion([
    #         point_base_frame.pose.orientation.x,
    #         point_base_frame.pose.orientation.y,
    #         point_base_frame.pose.orientation.z,
    #         point_base_frame.pose.orientation.w
    #     ])
    #     print("Orientation in base frame (roll, pitch, yaw): ", euler)

    def run(self):
        # Example point and orientation in the camera frame
        # self.transform_point(-0.1344, 0.1017, 0.6728, 146.3653*3.14/180, 5*3.14/180, 4*3.14/180)  # x, y, z, roll, pitch, yaw
        

        rospy.spin()

def main():
    translator = CoordinateTranslator()
    translator.run()

if __name__ == '__main__':
    main()

