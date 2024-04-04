import rospy
import tf2_ros
import tf_conversions
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState

class CoordinateTranslator:
    def __init__(self):
        rospy.init_node('coordinate_translator', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.aruco_pose_pub = rospy.Publisher("/aruco_pose_tf", PoseStamped, queue_size=1)
        self.aruco_sub = rospy.Subscriber("/aruco_pose", PoseStamped, self.aruco_callback)
    
    def publish_static_transformation(self, msg):

        # Create a StaticTransformBroadcaster object
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Create a TransformStamped message
        static_transform_stamped = TransformStamped()
        
        # Fill the message with the necessary data
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = '/xtion_optical_frame'
        static_transform_stamped.child_frame_id = '/aruco_marker_frame'
        static_transform_stamped.transform.translation = msg.pose.position
        
        static_transform_stamped.transform.rotation = msg.pose.orientation

        # Send the transformation
        static_broadcaster.sendTransform(static_transform_stamped)
        rospy.loginfo("Static transform published!")
        # rospy.loginfo("Sleeping for 2 seconds")
        # rospy.sleep(2)

    def aruco_callback(self, msg: PoseStamped):
        try:
            self.publish_static_transformation(msg)
            # Transform the pose to the target frame
            transform = self.tf_buffer.lookup_transform('torso_lift_link',  # e.g., 'base_link'
                                                        'aruco_marker_frame',  # Source frame
                                                        rospy.Time(0),  # Get the latest transform
                                                        rospy.Duration(1.0))  # Timeout
            transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)

            # Publish the transformed pose
            self.aruco_pose_pub.publish(transformed_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming ArUco marker pose: {e}")

    def run(self):
        rospy.spin()

def main():
    translator = CoordinateTranslator()
    translator.run()

if __name__ == '__main__':
    rospy.init_node('coordinate_translator', anonymous=True)
    main()

    

'''
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
        
        self.aruco_tf_pub = rospy.Publisher("/aruco_pose_tf", JointState, queue_size=1)

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
        self.marker_id = msg.name
        pose_vec = msg.position
        if len(self.aruco_pose) == 0:
            self.aruco_pose = pose_vec
            self.publish_static_transformation(pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5])
            # rospy.loginfo("Retrieving the transformation after 2 seconds!")
            # rospy.sleep(2)
            trans, rot, rot_euler = self.get_transformation()
            print("Point in base frame: ", trans)
            print("Orientation in base frame (roll, pitch, yaw): ", rot_euler)


            aruco_tf_msg = JointState()
            aruco_tf_msg.name = self.marker_id
            aruco_tf_msg.position = [trans[0], trans[1], trans[2], rot_euler[0], rot_euler[1], rot_euler[2]]
            self.aruco_tf_pub.publish(aruco_tf_msg)


    def get_transformation(self):
        #(trans, rot) = self.listener.lookupTransform("/base_footprint", "/aruco_marker_frame", rospy.Time(0))
        (trans, rot) = self.listener.lookupTransform("/torso_lift_link", "/aruco_marker_frame", rospy.Time(0))
        # print("Translation: ", trans)
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        # print("Rotation: ", rot_euler)
        # new_pose_dict = {"translation": trans, "rotation": rot}
        # return new_pose_dict
        return trans, rot, rot_euler

    def run(self):
        # Example point and orientation in the camera frame
     
        rospy.spin()

def main():
    translator = CoordinateTranslator()
    translator.run()

if __name__ == '__main__':
    main()
'''
