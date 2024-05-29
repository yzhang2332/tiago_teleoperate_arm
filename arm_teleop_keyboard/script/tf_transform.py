#!/usr/bin/env python
import rospy
import tf
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_conversions
from arm_teleop_keyboard.msg import PoseObj
import signal
import sys

class CoordinateTranslator:

    def __init__(self):

        rospy.init_node('coordinate_translator', anonymous=True)
        # self.tf_buffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.listener = tf.TransformListener()

        # self.aruco_sub = rospy.Subscriber("/aruco_pose", PoseStamped, self.aruco_callback)
        self.aruco_sub = rospy.Subscriber("/aruco_pose", PoseObj, self.aruco_callback)
        # self.aruco_pose_pub = rospy.Publisher("/aruco_pose_tf", PoseStamped, queue_size=1)
        self.aruco_pose_pub = rospy.Publisher("/aruco_pose_tf", PoseObj, queue_size=1)

        rospy.sleep(1)  # Wait for the listener to get ready

        self.aruco_pose = []


    def publish_static_transformation(self, msg):

        # Create a StaticTransformBroadcaster object
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Create a TransformStamped message
        static_transform_stamped = TransformStamped()
        
        # Fill the message with the necessary data
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = '/xtion_optical_frame'
        static_transform_stamped.child_frame_id = '/aruco_marker_frame'
        static_transform_stamped.transform.translation = msg.pose.pose.position
        
        static_transform_stamped.transform.rotation = msg.pose.pose.orientation

        # Send the transformation
        static_broadcaster.sendTransform(static_transform_stamped)
        # rospy.sleep(2)
        # rospy.loginfo("Static transform published!")


    def aruco_callback(self, msg: PoseObj):
        self.publish_static_transformation(msg)
        # self.get_transformation(msg)
        # self.aruco_pose_pub.publish(self.transformed_pose)

        trans, rot = self.get_transformation()  # Assuming get_transformation is adjusted to return translation and rotation
        object_pose = self.apply_transformation(msg, trans, rot)
        self.aruco_pose_pub.publish(object_pose)


    # def get_transformation(self, msg):
    #     marker_id = str(msg.header.frame_id)
    #     transform = self.tf_buffer.lookup_transform('torso_lift_link',  # e.g., 'base_link'
    #                                                 'aruco_marker_frame',  # Source frame
    #                                                 rospy.Time(0),  # Get the latest transform
    #                                                 rospy.Duration(1.0))  # Timeout
    #     transform.header.frame_id = f"torso_lift_link_{marker_id[-1]}"
        
    #     self.transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)

    def get_transformation(self):
        try:
            # Lookup the transformation
            # transform = self.tf_buffer.lookup_transform('torso_lift_link', 'aruco_marker_frame', rospy.Time(0))
            (trans, rot) = self.listener.lookupTransform('torso_lift_link', 'aruco_marker_frame', rospy.Time(0))
            
            # return transform.transform.translation, transform.transform.rotation
            return trans, rot
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Error obtaining transformation: %s", e)
            return None, None
    
    def apply_transformation(self, msg, trans, rot):
        # Here, we'd manually construct a new pose based on the transformation data
        # This is a placeholder to indicate where you would apply the transformation
        # For actual implementation, you'd need to use TF2's do_transform_pose or manually apply the transformation
        marker_id = msg.id
        transformed_pose = PoseStamped()
        transformed_pose.header = msg.pose.header
        transformed_pose.header.frame_id = "torso_lift_link"
        # transformed_pose.pose.position.x = trans.x + pose.pose.position.x
        # transformed_pose.pose.position.y = trans.y + pose.pose.position.y
        # transformed_pose.pose.position.z = trans.z + pose.pose.position.z
        transformed_pose.pose.position.x = trans[0]
        transformed_pose.pose.position.y = trans[1]
        transformed_pose.pose.position.z = trans[2]
        transformed_pose.pose.orientation.x = rot[0]
        transformed_pose.pose.orientation.y = rot[1]
        transformed_pose.pose.orientation.z = rot[2]
        transformed_pose.pose.orientation.w = rot[3]

        object_pose = PoseObj()
        object_pose.id = marker_id
        object_pose.pose = transformed_pose
        return object_pose

    def run(self):

        rospy.spin()

def signal_handler(sig, frame):
    print("Shutting down the ROS node...")
    rospy.signal_shutdown("Shutting down")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    translator = CoordinateTranslator()
    translator.run()

if __name__ == '__main__':
    main()

