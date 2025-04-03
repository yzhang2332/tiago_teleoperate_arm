#!/usr/bin/env python3

import rospy
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Vector, Rotation, JntArray
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from math import pi, inf
import csv
import sys
from pal_interaction_msgs.msg import TtsAction, TtsGoal

from geometry_msgs.msg import PoseStamped, Quaternion
import tf.transformations  # For converting between different representations
from std_msgs.msg import Float32MultiArray, String

import numpy as np


################# COSINE INTERPOLATION HELPERS #################
def cosine_interp_list(a, b, num_points):
    lst = []
    for i in range(len(a)):
        lst.append(cosine_interp_vals(a[i], b[i], num_points))
    return lst

def cosine_interp_vals(a, b, num_points):
    t = np.linspace(0, 1, num_points)
    # Transform t to the cosine space
    t = (1 - np.cos(t * np.pi)) / 2
    return [(1-tt) * a + tt * b for tt in t]



# Subscriber callback for updating current joint positions
def joint_state_callback(msg):
    global current_joint_positions, current_gripper_position, current_head_position
    try:
        for i, name in enumerate(joint_names):
            index = msg.name.index(name)
            current_joint_positions[i] = msg.position[index]
        gripper_index = msg.name.index("gripper_left_finger_joint")
        current_gripper_position[0] = msg.position[gripper_index]*2
        current_gripper_position[1] = msg.position[gripper_index]*2
        head_1_index = msg.name.index("head_1_joint")
        head_2_index = msg.name.index("head_2_joint")
        current_head_position[0] = msg.position[head_1_index]
        current_head_position[1] = msg.position[head_2_index]
    except Exception as e:
        rospy.logerr(f"Error in joint_state_callback: {e}")

# Your other functions (update_gripper_position, update_head_position, apply_joint_positions, get_current_end_effector_pose) remain the same


def update_gripper_position(increment):
    global current_gripper_position

    # Update the current position based on the increment/decrement value
    # new_position = [pos + increment for pos in current_gripper_position]
    new_position = [increment, increment]

    # Ensure the new position is within the allowable range
    # Assuming the gripper range is between 0 (fully closed) and 0.04 (fully open)
    new_position = [max(0.02, min(1, pos)) for pos in new_position]

    # Update the global variable
    current_gripper_position = new_position

    # Create and send the new goal to the action server
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
    point = JointTrajectoryPoint()
    point.positions = current_gripper_position
    point.time_from_start = rospy.Duration(0.5)
    trajectory.points.append(point)
    goal.trajectory = trajectory
    
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()


def update_head_position(pan_degrees, tilt_degrees, duration):
    global current_head_position
    
    # Convert degrees to radians for ROS
    pan_radians = pan_degrees * 3.141592653589793 / 180.0
    tilt_radians = tilt_degrees * 3.141592653589793 / 180.0

    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['head_1_joint', 'head_2_joint']
    point = JointTrajectoryPoint()
    point.positions =  [pan_radians, tilt_radians] 
    point.time_from_start = rospy.Duration(duration)
    trajectory.points.append(point)
    goal.trajectory = trajectory
    
    head_client.send_goal(goal)
    head_client.wait_for_result()


def apply_joint_positions_with_interp(joint_position_dict, duration):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    #traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names

    start_joint_list = [current_joint_positions[i] for i in range(number_of_joints)]
    end_joint_list = [0] * len(joint_names)
    for i, name in enumerate(joint_names):
        end_joint_list[i] = joint_position_dict[name]

    num_interp = 100
    interp_lists = cosine_interp_list(start_joint_list, end_joint_list, num_interp)

    # add all joint lists
    for i in range(num_interp):

        this_joint_list = []
        for joint_index in range(len(interp_lists)):
            this_joint_list.append(interp_lists[joint_index][i])
    
        point = JointTrajectoryPoint()
        
        point.positions = this_joint_list
        # print(duration/(num_interp-1))
        point.time_from_start = rospy.Duration(i*(duration/(num_interp-1)))  # Adjust based on your requirements
        traj_msg.points.append(point)

    # print("message", traj_msg)
    
    # Publish the message
    arm_pub.publish(traj_msg)


def apply_joint_positions(joint_position_dict, duration):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    #traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names
    
    point = JointTrajectoryPoint()

    all_position = [0] * len(joint_names)

    for i, name in enumerate(joint_names):
        all_position[i] = joint_position_dict[name]
    
    point.positions = all_position
    point.time_from_start = rospy.Duration(duration)  # Adjust based on your requirements
    traj_msg.points.append(point)

    # print("message", traj_msg)
    
    # Publish the message
    arm_pub.publish(traj_msg)

# Function to get the current pose of the end-effector
def get_current_end_effector_pose():
    current_pose = Frame()
    fk_solver.JntToCart(current_joint_positions, current_pose)
    return current_pose



def move_to_goal_position(goal_position, goal_orientation, duration):
    global desired_frame, current_joint_positions, ik_solver_pos
    # Set desired frame based on the goal position and orientation
    desired_position = Vector(*goal_position)
    desired_orientation = Rotation.RPY(*goal_orientation)
    desired_frame = Frame(desired_orientation, desired_position)
    
    # Use IK to calculate desired joint positions
    ik_result = ik_solver_pos.CartToJnt(current_joint_positions, desired_frame, desired_joint_positions)
    if ik_result >= 0:  # If IK solution is found
        joint_positions_dict = {joint_names[i]: desired_joint_positions[i] for i in range(number_of_joints)}
        # apply_joint_positions(joint_positions_dict, duration)
        apply_joint_positions_with_interp(joint_positions_dict, duration)
    else:
        rospy.logerr("Failed to find an IK solution for the desired position.")




class GenerationFunction():
    def __init__(self, message):
        self.message = message
        self.client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.client.wait_for_server()
        rospy.sleep(0.1)
        self.goal = TtsGoal()
    def speak(self):
        try:
            self.goal.rawtext.text = self.message
            self.goal.rawtext.lang_id = "en_GB"
            self.client.send_goal_and_wait(self.goal)
        except Exception as e:
            rospy.logerr(f"Failed to say Hi: {e}")
            return "Error trying to say Hi."


def move_arm(joint_angles, t):
        # Define the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        # Specify the joint names for arm and torso
        trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Define the joint target positions for arm and torso
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(t)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory

        # Send the goal and wait for the result
        rospy.loginfo("Sending goal for arm and torso movement...")
        arm_client.send_goal(goal)
        if arm_client.wait_for_result(rospy.Duration(t+1)):  # Increase timeout to ensure enough time for execution
            rospy.loginfo("Arm completed successfully.")
        else:
            rospy.loginfo("Arm did not complete before the timeout.")


def move_arm_with_interp(joint_angles_list, duration):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    #traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names

    start_joint_list = [current_joint_positions[i] for i in range(number_of_joints)]
    end_joint_list = joint_angles_list

    num_interp = 100
    interp_lists = cosine_interp_list(start_joint_list, end_joint_list, num_interp)

    # add all joint lists
    for i in range(num_interp):

        this_joint_list = []
        for joint_index in range(len(interp_lists)):
            this_joint_list.append(interp_lists[joint_index][i])
    
        point = JointTrajectoryPoint()
        
        point.positions = this_joint_list
        point.time_from_start = rospy.Duration(i*(duration/(num_interp-1)))  # Adjust based on your requirements
        traj_msg.points.append(point)

    # print("message", traj_msg)
    
    # Publish the message
    arm_pub.publish(traj_msg)


def aruco_pose_callback(msg):
    global Arco_markers_updated_array_1, Arco_markers_updated_array_2
    if msg.name[0] == "1":
        Arco_markers_updated_array_1 = np.array([msg.name[0], msg.position[0],msg.position[1]], dtype=object)
    elif msg.name[0] == "2":
        Arco_markers_updated_array_2 = np.array([msg.name[0], msg.position[0],msg.position[1]], dtype=object)
    


def send_audio(text):
    # rospy.init_node("audio_sender", anonymous=True)
    # pub = rospy.Publisher("/play_audio", String, queue_size=10)
    
    # Wait for ROS Master to start
    # rospy.sleep(1)

    # Path to the WAV file on the robot
    audio_file = "/home/pal/Audios/"+text+".wav"
    
    rospy.loginfo(f"📤 Sending audio file path: {audio_file}")
    Audio_pub.publish(audio_file)


# Subscribe to the raw image topic
def image_callback(msg):
    if publish_images:
        filtered_pub.publish(msg)
        rospy.loginfo("Publishing image...")

def PositionGiver(Cube_numberr, row, X_forward, state):
    if Cube_numberr==1:
        X_pose = 0.58
        Y_pose = 0.11
    elif Cube_numberr == 2:
        X_pose = 0.58
        Y_pose = -0.005
    elif Cube_numberr == 3:
        X_pose = 0.58
        Y_pose = -0.115
    elif Cube_numberr == 4:
        X_pose = 0.58
        Y_pose = -0.115*2+0.005
    elif Cube_numberr == 5:
        X_pose = 0.58
        Y_pose = -0.115*3+0.005

    if row == 2:
        X_pose = X_pose + X_forward
    
    return X_pose, Y_pose

def PositionUpdater(X_aruco, Y_aruco, Cube_number, row, movement, State = "Normal"):
    if movement == "Drop":
        if Cube_number == 1 and row == 1:
            X_update = -(349-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 2 and row == 1:
            X_update = -(349-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 3 and row == 1:
            X_update = -(320-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 4 and row == 1:
            X_update = -(250-Y_aruco)*0.01/65+0.002
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 5 and row == 1:
            X_update = -(230-Y_aruco)*0.01/65+0.002
            Y_update = (209-X_aruco)*0.015/82

        elif Cube_number == 1 and row == 2:
            X_update = -(320-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 2 and row == 2:
            X_update = -(300-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 3 and row == 2:
            X_update = -(310-Y_aruco)*0.006/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 4 and row == 2:
            X_update = -(220-Y_aruco)*0.007/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 5 and row == 2:
            X_update = -(170-Y_aruco)*0.01/65 -0.015
            Y_update = (209-X_aruco)*0.013/82

    elif State == "Droped":
        if Cube_number == 1 and row == 1:
            X_update = -(349-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 2 and row == 1:
            X_update = -(349-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 3 and row == 1:
            X_update = -(320-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 4 and row == 1:
            X_update = -(250-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 5 and row == 1:
            X_update = -(200-Y_aruco)*0.01/65 +0.003
            Y_update = (209-X_aruco)*0.013/82

        elif Cube_number == 1 and row == 2:
            X_update = -(320-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 2 and row == 2:
            X_update = -(300-Y_aruco)*0.01/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 3 and row == 2:
            X_update = -(310-Y_aruco)*0.006/65+0.002
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 4 and row == 2:
            X_update = -(220-Y_aruco)*0.007/65
            Y_update = (209-X_aruco)*0.015/82
        elif Cube_number == 5 and row == 2:
            X_update = -(220-Y_aruco)*0.01/65 -0.01
            Y_update = (209-X_aruco)*0.013/82

    elif State == "Stacked":
        if Cube_number == 1 and row == 1:
            X_update = (203-X_aruco)*0.013/82+0.004
            Y_update = (343-Y_aruco)*0.012/65
        elif Cube_number == 2 and row == 1:
            X_update = (203-X_aruco)*0.013/82+0.004
            Y_update = (343-Y_aruco)*0.012/65
        elif Cube_number == 3 and row == 1:
            X_update = (203-X_aruco)*0.013/82
            Y_update = (343-Y_aruco)*0.012/65
        elif Cube_number == 4 and row == 1:
            X_update = (203-X_aruco)*0.013/82
            Y_update = (343-Y_aruco)*0.012/65
        elif Cube_number == 5 and row == 1:
            X_update = (203-X_aruco)*0.013/82
            Y_update = (343-Y_aruco)*0.012/65

        # elif Cube_number == 4 and row == 1:
        #     X_update = (203-X_aruco)*0.013/82
        #     Y_update = (343-Y_aruco)*0.012/65
        # elif Cube_number == 5 and row == 1:
        #     X_update = (280-X_aruco)*0.013/82+0.004
        #     Y_update = (343-Y_aruco)*0.010/65-0.005

        elif Cube_number == 1 and row == 2:
            X_update = (200-X_aruco)*0.013/82+0.004
            Y_update = (343-Y_aruco)*0.010/65
        elif Cube_number == 2 and row == 2:
            X_update = (200-X_aruco)*0.013/82+0.004
            Y_update = (343-Y_aruco)*0.010/65
        elif Cube_number == 3 and row == 2:
            X_update = (200-X_aruco)*0.013/82
            Y_update = (343-Y_aruco)*0.010/65
        elif Cube_number == 4 and row == 2:
            X_update = (200-X_aruco)*0.013/82-0.013
            Y_update = (343-Y_aruco)*0.010/65
        elif Cube_number == 5 and row == 2:
            X_update = (250-X_aruco)*0.013/82+0.015-0.02
            Y_update = (343-Y_aruco)*0.010/65

    else: 
        if Cube_number == 1 and row == 1:
            X_update = (203-X_aruco)*0.013/82+0.004
            Y_update = (343-Y_aruco)*0.012/65
        elif Cube_number == 2 and row == 1:
            X_update = (203-X_aruco)*0.013/82+0.004
            Y_update = (343-Y_aruco)*0.012/65
        elif Cube_number == 3 and row == 1:
            X_update = (203-X_aruco)*0.013/82
            Y_update = (343-Y_aruco)*0.012/65
        elif Cube_number == 4 and row == 1:
            X_update = (203-X_aruco)*0.013/82
            Y_update = (343-Y_aruco)*0.012/65
        elif Cube_number == 5 and row == 1:
            X_update = (203-X_aruco)*0.013/82
            Y_update = (343-Y_aruco)*0.012/65

        # elif Cube_number == 4 and row == 1:
        #     X_update = (203-X_aruco)*0.013/82
        #     Y_update = (343-Y_aruco)*0.012/65
        # elif Cube_number == 5 and row == 1:
        #     X_update = (280-X_aruco)*0.013/82+0.004
        #     Y_update = (343-Y_aruco)*0.010/65-0.005

        elif Cube_number == 1 and row == 2:
            X_update = (200-X_aruco)*0.013/82+0.004
            Y_update = (343-Y_aruco)*0.010/65
        elif Cube_number == 2 and row == 2:
            X_update = (200-X_aruco)*0.013/82+0.004
            Y_update = (343-Y_aruco)*0.010/65
        elif Cube_number == 3 and row == 2:
            X_update = (200-X_aruco)*0.013/82
            Y_update = (343-Y_aruco)*0.010/65
        elif Cube_number == 4 and row == 2:
            X_update = (200-X_aruco)*0.013/82
            Y_update = (343-Y_aruco)*0.010/65
        elif Cube_number == 5 and row == 2:
            X_update = (250-X_aruco)*0.013/82+0.004
            Y_update = (343-Y_aruco)*0.010/65


        # elif Cube_number == 4 and row == 2:
        #     X_update = (250-X_aruco)*0.013/82+0.004
        #     Y_update = (343-Y_aruco)*0.010/65
        # elif Cube_number == 5 and row == 2:
        #     X_update = (370-X_aruco)*0.013/82+0.004
        #     Y_update = (343-Y_aruco)*0.010/65-0.008
    return X_update, Y_update


def run():
    global ik_solver_pos, desired_joint_positions, joint_names, number_of_joints, fk_solver, arm_pub, gripper_client, desired_frame, current_gripper_position, current_joint_positions, current_head_position, head_client, ee_pose_pub, State_pub, arm_client, Audio_pub
    global data, filtered_pub, publish_images

    publish_images = False


    # Load the robot model from the parameter server
    robot_urdf = URDF.from_parameter_server()

    # Generate a KDL tree from the URDF model
    success, kdl_tree = treeFromUrdfModel(robot_urdf)
    if not success:
        rospy.logerr("Failed to extract KDL tree from URDF robot model.")
        exit(1)

    # Specify the chain: from base link to end-effector link
    base_link = "torso_lift_link"
    end_effector_link = "gripper_link"

    chain = kdl_tree.getChain(base_link, end_effector_link)

    # Initialize the IK solver
    ik_solver_pos = ChainIkSolverPos_LMA(chain)

    # Initialize the joint array with the number of joints
    number_of_joints = chain.getNrOfJoints()
    desired_joint_positions = JntArray(number_of_joints)
    current_joint_positions = JntArray(number_of_joints)

    # Initialize Forward Kinematics solver
    fk_solver = ChainFkSolverPos_recursive(chain)

    # List of joint names for the robot's arm - adjust this list to match your configuration
    joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
                   "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]

    current_gripper_position = [0, 0]
    current_head_position = [0, 0]

    # Publisher for controlling the robot's arm
    arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    Audio_pub = rospy.Publisher("/play_audio", String, queue_size=10)

    # Action clients for the gripper and head
    gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()

    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("arm server connected.")

    # Subscribe to the current joint state
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.loginfo("Subscribed to joint states.")

    rospy.Subscriber('/aruco_pose_robot_camera', JointState, aruco_pose_callback)
    rospy.loginfo("Subscribed to aruco pose")

    # Wait to get first values to ensure everything is initialized properly
    rospy.wait_for_message('/joint_states', JointState)
    rospy.sleep(1.0)

    # Add a global publisher for end-effector pose
    ee_pose_pub = rospy.Publisher('/end_effector_pose', PoseStamped, queue_size=1)
    State_pub = rospy.Publisher('/ee_state_topic', Float32MultiArray, queue_size=1)

    filtered_pub = rospy.Publisher("/filtered_image_raw", Image, queue_size=10)
    rospy.Subscriber("/xtion/rgb/image_raw", Image, image_callback)



    if len(sys.argv) > 1:
        Cube_number = int(sys.argv[1])
        movement = sys.argv[2]
        row = int(sys.argv[3])
        State = sys.argv[4]
        if movement == "Move" or movement == "Unstack" or movement == "Stack":
            move_target = int(sys.argv[5])
            row_target = int(sys.argv[6])

    # object_number = System_input % 4

    #First row
    #First one 

   

    #parameters
    X_forward = 0.07
    y_left = -0.05
    X_Play = 0.75
    Y_play = -0.2
    X_discard = 0.7
    Y_discard = 0.3

    X_pose, Y_pose = PositionGiver(Cube_number, row, X_forward, State)



    #Step 2 Openning Gripper
    update_gripper_position(0.09)
    rospy.sleep(0.7)

    if movement == "Stack":
        X_pose_stack, Y_pose_stack = PositionGiver(move_target, row_target, X_forward, "Normal")
        # Step 1 Going upper of the object
        goal_orientation = [0, 0, pi/2]  # Adjust as needed

        goal_position = [X_pose_stack, Y_pose_stack, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 4)
        rospy.sleep(5)


        goal_position = [X_pose_stack, Y_pose_stack, -0.20]  # Adjust as needed
        if Cube_number == 5 and row == 2:
            goal_position = [X_pose-0.01, Y_pose, -0.20]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(3)
        #Step 3 Going Down
        Aruco_id = Arco_markers_updated_array_1[0]
        X_aruco = Arco_markers_updated_array_1[2]
        Y_aruco = Arco_markers_updated_array_1[1]
        X_update, Y_update = PositionUpdater(X_aruco, Y_aruco, Cube_number, row, movement, "Normal")
        X_pose_stack = X_pose_stack + X_update
        Y_pose_stack = Y_pose_stack + Y_update

        goal_position = [X_pose_stack, Y_pose_stack, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 3)
        rospy.sleep(3)


    if State == "Droped":

        # Step 1 Going upper of the object
        goal_orientation = [0, 0, pi]  # Adjust as needed
        goal_position = [X_pose, Y_pose+0.015, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 4)
        rospy.sleep(4)


        goal_position = [X_pose, Y_pose+0.015, -0.20]  # Adjust as needed
        if Cube_number == 5 and row == 2:
            goal_position = [X_pose-0.015, Y_pose+0.015, -0.20]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(3)

        #Step 3 Going Down
        Aruco_id = Arco_markers_updated_array_2[0]
        X_aruco = Arco_markers_updated_array_2[2]
        Y_aruco = Arco_markers_updated_array_2[1]


        X_update, Y_update = PositionUpdater(X_aruco, Y_aruco, Cube_number, row, movement, "Droped")


        goal_position = [X_pose+X_update, Y_pose+Y_update+0.023, -0.26]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 3)
        rospy.sleep(3)

        #Step 2 Openning Gripper
        update_gripper_position(0.035)
        rospy.sleep(0.7)

        # Step 1 Going upper of the object
        goal_orientation = [0, 0, pi]  # Adjust as needed
        goal_position = [X_pose, Y_pose, -0.21]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 4)
        rospy.sleep(5)

        # Step 1 Going upper of the object
        goal_orientation = [0, 0, pi]  # Adjust as needed
        goal_position = [X_pose, Y_pose, -0.23]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 1)
        rospy.sleep(1)

        #Step 2 Openning Gripper
        update_gripper_position(0.04)
        rospy.sleep(0.7)

        goal_position = [X_pose, Y_pose, -0.248]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 3)
        rospy.sleep(3)

        #Step 2 Openning Gripper
        update_gripper_position(0.09)
        rospy.sleep(0.7)

        goal_position = [X_pose, Y_pose,-0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

    elif State == "Stacked":
        # Step 1 Going upper of the object
        goal_orientation = [0, 0, pi/2]  # Adjust as needed
        goal_position = [X_pose, Y_pose, -0.13]  # Adjust as needed
        if Cube_number == 5 and row == 2:
            goal_position = [X_pose-0.02, Y_pose, -0.13]  # Adjust as needed
        if Cube_number == 4 and row == 2:
            goal_position = [X_pose-0.02, Y_pose, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 3)
        rospy.sleep(4)

        #Step 3 Going Down
        Aruco_id = Arco_markers_updated_array_1[0]
        X_aruco = Arco_markers_updated_array_1[2]
        Y_aruco = Arco_markers_updated_array_1[1]

        X_update, Y_update = PositionUpdater(X_aruco, Y_aruco, Cube_number, row, movement, "Stacked")
    
 
        goal_position = [X_pose+X_update, Y_pose+Y_update, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 1)
        rospy.sleep(1)

        goal_position = [X_pose+X_update, Y_pose+Y_update, -0.202]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)


        # goal_position = [X_pose+X_update, Y_pose+Y_update, -0.248]  # Adjust as needed
        # if movement == "Move" or movement == "Stack" or movement == "Discard" or movement == "Play":
        #     goal_position = [X_pose+X_update, Y_pose+Y_update, -0.27]  # Adjust as needed
        # move_to_goal_position(goal_position, goal_orientation, 3)
        # rospy.sleep(3)

        # #Step 2 Openning Gripper
 
        update_gripper_position(0.05)
        rospy.sleep(0.7)

        goal_position = [X_pose, Y_pose,-0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)


    if movement != "Stand" and State != "Stacked":

        # Step 1 Going upper of the object
        if movement == "Drop":
            goal_orientation = [0, 0, pi]  # Adjust as needed
        else:
            goal_orientation = [0, 0, pi/2]  # Adjust as needed
        goal_position = [X_pose, Y_pose, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 3)
        rospy.sleep(3)

        goal_position = [X_pose, Y_pose, -0.20]  # Adjust as needed
        if Cube_number == 5 and row == 2 and movement == "Drop":
            goal_position = [X_pose-0.01, Y_pose, -0.20]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(3)

        #Step 3 Going Down
        Aruco_id = Arco_markers_updated_array_1[0]
        X_aruco = Arco_markers_updated_array_1[2]
        Y_aruco = Arco_markers_updated_array_1[1]

        X_update, Y_update = PositionUpdater(X_aruco, Y_aruco, Cube_number, row, movement)
    
 
        if movement != "Drop":
            goal_position = [X_pose+X_update, Y_pose+Y_update, -0.20]  # Adjust as needed
            move_to_goal_position(goal_position, goal_orientation, 1)
            rospy.sleep(1)

        goal_position = [X_pose+X_update, Y_pose+Y_update, -0.27]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 3)
        rospy.sleep(3)

        #Step 2 Openning Gripper
        if movement == "Drop":
            update_gripper_position(0.035)
        else:
            update_gripper_position(0.05)
        rospy.sleep(0.7)

    # if movement == "Unstack":
    #     # Step 1 Going upper of the object
    #     goal_orientation = [0, 0, pi/2]  # Adjust as needed
    #     goal_position = [X_pose, Y_pose, -0.13]  # Adjust as needed
    #     move_to_goal_position(goal_position, goal_orientation, 3)
    #     rospy.sleep(3)

    #     # goal_position = [X_pose, Y_pose, -0.20]  # Adjust as needed
    #     # if Cube_number == 5 and row == 2 and movement == "Drop":
    #     #     goal_position = [X_pose-0.01, Y_pose, -0.20]  # Adjust as needed
    #     # move_to_goal_position(goal_position, goal_orientation, 2)
    #     # rospy.sleep(3)

    #     #Step 3 Going Down
    #     Aruco_id = Arco_markers_updated_array_1[0]
    #     X_aruco = Arco_markers_updated_array_1[2]
    #     Y_aruco = Arco_markers_updated_array_1[1]

    #     X_update, Y_update = PositionUpdater(X_aruco, Y_aruco, Cube_number, row, movement)
    
 
    #     goal_position = [X_pose+X_update+0.002, Y_pose+Y_update, -0.205]  # Adjust as needed
    #     move_to_goal_position(goal_position, goal_orientation, 2)
    #     rospy.sleep(2)


    #     # goal_position = [X_pose+X_update, Y_pose+Y_update, -0.248]  # Adjust as needed
    #     # if movement == "Move" or movement == "Stack" or movement == "Discard" or movement == "Play":
    #     #     goal_position = [X_pose+X_update, Y_pose+Y_update, -0.27]  # Adjust as needed
    #     # move_to_goal_position(goal_position, goal_orientation, 3)
    #     # rospy.sleep(3)

    #     # #Step 2 Openning Gripper
 
    #     update_gripper_position(0.05)
    #     rospy.sleep(0.7)




    if movement == "Forward":
        if State != "Stacked":
            goal_position = [X_pose+X_update, Y_pose+Y_update, -0.13]  # Adjust as needed
            move_to_goal_position(goal_position, goal_orientation, 2)
            rospy.sleep(2)

        goal_position = [X_pose+X_forward, Y_pose, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        goal_position = [X_pose+X_forward, Y_pose, -0.265]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(3)

        #Step 2 Openning Gripper
        update_gripper_position(0.09)
        rospy.sleep(0.7)

        goal_position = [X_pose+X_forward, Y_pose,-0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(0.3)
        up_joint_angles = [0.07, 0.68, -1.77, 1.98, 0.68, -1.39, 0.49]
        move_arm_with_interp(up_joint_angles, 5)


    elif movement == "Backward":
        if State != "Stacked":
            goal_position = [X_pose+X_update, Y_pose+Y_update, -0.13]  # Adjust as needed
            move_to_goal_position(goal_position, goal_orientation, 2)
            rospy.sleep(2)

        goal_position = [X_pose-X_forward, Y_pose, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        goal_position = [X_pose-X_forward, Y_pose, -0.265]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(3)

        #Step 2 Openning Gripper
        update_gripper_position(0.09)
        rospy.sleep(0.7)

        goal_position = [X_pose-X_forward, Y_pose, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(0.3)
        up_joint_angles = [0.07, 0.68, -1.77, 1.98, 0.68, -1.39, 0.49]
        move_arm_with_interp(up_joint_angles, 5)



    elif movement == "Drop":

        goal_position = [X_pose+X_update, Y_pose+Y_update, -0.24]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 1)
        rospy.sleep(1)

        goal_position = [X_pose, Y_pose+y_left, -0.24]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        goal_position = [X_pose, Y_pose+y_left, -0.248]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 1)
        rospy.sleep(1)

        goal_position = [X_pose, Y_pose+y_left+0.03, -0.248]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 1)
        rospy.sleep(1)

        goal_position = [X_pose-0.001, Y_pose+y_left+0.05, -0.26]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 1)
        rospy.sleep(1)

        goal_position = [X_pose-0.001, Y_pose+y_left+0.06, -0.27]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 1)
        rospy.sleep(1)

        #Step 2 Openning Gripper
        update_gripper_position(0.05)
        rospy.sleep(0.7)

        goal_position = [X_pose-0.002, Y_pose+y_left+0.06, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        #Step 2 Openning Gripper
        update_gripper_position(0.09)
        rospy.sleep(0.7)


        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(0.3)
        up_joint_angles = [0.07, 0.68, -1.77, 1.98, 0.68, -1.39, 0.49]
        move_arm_with_interp(up_joint_angles, 5)


    elif movement == "Play":
        if State != "Stacked":
            goal_position = [X_pose+X_update, Y_pose+Y_update, -0.13]  # Adjust as needed
            move_to_goal_position(goal_position, goal_orientation, 2)
            rospy.sleep(2)

        goal_position = [X_Play, Y_play, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 4)
        rospy.sleep(4)

        goal_position = [X_Play, Y_play, -0.265]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        #Step 2 Openning Gripper
        update_gripper_position(0.09)
        rospy.sleep(0.7)

        goal_position = [X_Play, Y_play, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(0.3)
        up_joint_angles = [0.07, 0.68, -1.77, 1.98, 0.68, -1.39, 0.49]
        move_arm_with_interp(up_joint_angles, 5)

    elif movement == "Discard":
        if State != "Stacked":
            goal_position = [X_pose+X_update, Y_pose+Y_update, -0.13]  # Adjust as needed
            move_to_goal_position(goal_position, goal_orientation, 2)
            rospy.sleep(2)


        # goal_position = [X_pose+X_update, Y_pose+Y_update, -0.2]  # Adjust as needed
        # move_to_goal_position(goal_position, goal_orientation, 1)
        # rospy.sleep(1)

        goal_position = [X_discard, Y_discard, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 4)
        rospy.sleep(4)

        goal_position = [X_discard, Y_discard, -0.26]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        #Step 2 Openning Gripper
        update_gripper_position(0.09)
        rospy.sleep(0.7)

        goal_position = [X_discard, Y_discard, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(0.3)
        up_joint_angles = [0.07, 0.68, -1.77, 1.98, 0.68, -1.39, 0.49]
        move_arm_with_interp(up_joint_angles, 5)


    elif movement == "Move" or movement == "Unstack":
        if State != "Stacked":
            goal_position = [X_pose+X_update, Y_pose+Y_update, -0.13]  # Adjust as needed
            move_to_goal_position(goal_position, goal_orientation, 3)
            rospy.sleep(3)

        X_pose, Y_pose = PositionGiver(move_target, row_target, X_forward, State)


        goal_position = [X_pose, Y_pose, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 4)
        rospy.sleep(4)

        goal_position = [X_pose, Y_pose, -0.27]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        #Step 2 Openning Gripper
        update_gripper_position(0.09)
        rospy.sleep(0.9)

        goal_position = [X_pose, Y_pose,-0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(0.3)
        up_joint_angles = [0.07, 0.68, -1.77, 1.98, 0.68, -1.39, 0.49]
        move_arm_with_interp(up_joint_angles, 5)

    elif movement == "Stack":

        goal_position = [X_pose+X_update, Y_pose+Y_update, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 3)
        rospy.sleep(3)

        # X_pose, Y_pose = PositionGiver(move_target, row_target, X_forward, State)
        X_pose = X_pose_stack
        Y_pose = Y_pose_stack
        

        goal_position = [X_pose, Y_pose, -0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 4)
        rospy.sleep(4)

        goal_position = [X_pose, Y_pose, -0.204]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 3)
        rospy.sleep(3)

        #Step 2 Openning Gripper
        update_gripper_position(0.09)
        rospy.sleep(0.9)

        # goal_position = [X_pose+0.07, Y_pose,-0.2]  # Adjust as needed
        # move_to_goal_position(goal_position, goal_orientation, 2)
        # rospy.sleep(2)

        # goal_position = [0.7, -0.2,-0.2]  # Adjust as needed
        # move_to_goal_position(goal_position, goal_orientation, 4)
        # rospy.sleep(4)

        # rospy.wait_for_message("joint_states", JointState)
        # rospy.sleep(0.3)
        # up_joint_angles = [0.07, 0.44, -1.57, 1.98, 0.39, -1.39, 0.34]
        # move_arm_with_interp(up_joint_angles, 5)

        goal_position = [X_pose, Y_pose,-0.13]  # Adjust as needed
        move_to_goal_position(goal_position, goal_orientation, 2)
        rospy.sleep(2)

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(0.3)
        up_joint_angles = [0.07, 0.68, -1.77, 1.98, 0.68, -1.39, 0.49]
        move_arm_with_interp(up_joint_angles, 5)


    elif movement == "Stand":

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(0.3)
        up_joint_angles = [0.07, 0.68, -1.77, 1.98, 0.68, -1.39, 0.49]
        move_arm_with_interp(up_joint_angles, 5)

        


    # Get current end-effector position and orientation
    current_end_effector_pose = get_current_end_effector_pose()
    current_position = current_end_effector_pose.p
    current_orientation = current_end_effector_pose.M

    
    print(f"x: {current_position.x()}, y: {current_position.y()}, z: {current_position.z()}")

    # Calculate the new orientation by converting to RPY, adding the deltas, and converting back
    roll, pitch, yaw = current_orientation.GetRPY()

    print(f"Orientation - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")



if __name__ == "__main__":
    global data
    # Initialize ROS node
    rospy.init_node('tiago_arm_teleop_position')
    run()

