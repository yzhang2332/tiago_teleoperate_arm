#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Rotation, Vector, JntArray
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState

class ArucoArmController:
    def __init__(self):
        rospy.init_node('aruco_arm_controller')

        # Initialize robot arm control components
        self.initialize_robot_arm()

        # Setup TF2 listener for transforms - not directly used here but useful if you have additional transforms.
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Action client for sending joint trajectories
        self.arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm controller server...")
        self.arm_client.wait_for_server()
        rospy.loginfo("Arm controller server found.")
        
        # Subscriber for ArUco marker poses
        self.aruco_sub = rospy.Subscriber("/aruco_pose_tf", PoseStamped, self.aruco_pose_callback)

        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.wait_for_message("aruco_pose_tf", PoseStamped)
        rospy.sleep(2.0)
        rospy.loginfo("Get the joint states message")
        self.move_arm()
        
        
    def initialize_robot_arm(self):
        # Load URDF, setup KDL tree, and prepare IK and FK solvers
        robot_urdf = URDF.from_parameter_server()
        _, kdl_tree = treeFromUrdfModel(robot_urdf)
        
        # Update these names to match your robot configuration
        base_link = "torso_lift_link"
        end_effector_link = "gripper_grasping_frame_Z"
        # end_effector_link = "gripper_link"
        
        self.chain = kdl_tree.getChain(base_link, end_effector_link)
        self.ik_solver = ChainIkSolverPos_LMA(self.chain)
        self.current_joint_positions = JntArray(self.chain.getNrOfJoints())
        self.joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
                "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
                ]  # Update with actual joint names
        
    
    def joint_state_callback(self, msg):
        try:
            for i, name in enumerate(self.joint_names):
                index = msg.name.index(name)
                self.current_joint_positions[i] = msg.position[index]

        except ValueError as e:
            rospy.logerr(f"Error in joint_state_callback: {e}")
    
    def pose_to_kdl_frame(self, pose):
        orientation = pose.orientation
        return Frame(Rotation.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
                     Vector(pose.position.x, pose.position.y, pose.position.z))
    
    def aruco_pose_callback(self, msg):
        # rospy.loginfo("Received ArUco marker pose in frame: %s", msg.header.frame_id)
        self.aruco_msg = msg
        

    def move_arm(self):
        target_frame = self.pose_to_kdl_frame(self.aruco_msg.pose)
        
        desired_joint_positions = JntArray(self.chain.getNrOfJoints())
        
        if self.ik_solver.CartToJnt(self.current_joint_positions, target_frame, desired_joint_positions) >= 0:
            positions = [desired_joint_positions[i] for i in range(desired_joint_positions.rows())]
            self.send_trajectory(positions)
        else:
            rospy.logerr("Failed to find an IK solution")

    def send_trajectory(self, positions):
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(2)  # Modify as needed
        trajectory.points.append(point)
        goal.trajectory = trajectory
        self.arm_client.send_goal(goal)
        print("goal", goal)
        self.arm_client.wait_for_result(rospy.Duration(5))  # Modify as needed
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = ArucoArmController()
    controller.run()
