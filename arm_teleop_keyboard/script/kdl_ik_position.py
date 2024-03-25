#!/usr/bin/env python3

import rospy
import time
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Vector, Rotation, JntArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pynput.keyboard import Key, Listener
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import SwitchController

# def switch_controllers():
#     rospy.wait_for_service('/controller_manager/switch_controller')
#     try:
#         switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
#         resp = switch_controller(start_controllers=['arm_controller'], stop_controllers=[], strictness=0)
#         return resp.ok # Check if the service call was successful
#     except rospy.ServiceException as e:
#         print("Service call failed: %s" % e)

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


def update_gripper_position(increment):
    global current_gripper_position

    # Update the current position based on the increment/decrement value
    new_position = [pos + increment for pos in current_gripper_position]

    # Ensure the new position is within the allowable range
    # Assuming the gripper range is between 0 (fully closed) and 0.04 (fully open)
    new_position = [max(0, min(1, pos)) for pos in new_position]

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

def update_head_position(pan_increment=0.0, tilt_increment=0.0):
    global current_head_position
    
    current_head_position[0] += pan_increment
    current_head_position[1] += tilt_increment
    
    current_head_position[0] = max(-1.25, min(1.25, current_head_position[0]))
    current_head_position[1] = max(-1.25, min(1.25, current_head_position[1]))

    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['head_1_joint', 'head_2_joint']
    point = JointTrajectoryPoint()
    point.positions = current_head_position
    point.time_from_start = rospy.Duration(0.5)
    trajectory.points.append(point)
    goal.trajectory = trajectory
    
    head_client.send_goal(goal)
    head_client.wait_for_result()
    
    
def apply_joint_positions(c, joint_position_dict):
    # Create a JointTrajectory message
    duration = 0.5
    traj_msg = JointTrajectory()
    # traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names

    # start_point = JointTrajectoryPoint()
    # start_point.positions = [c[0], c[1], c[2], c[3], c[4], c[5], c[6]]
    # start_point.time_from_start = rospy.Duration(0)
    # traj_msg.points.append(start_point)
    
    point = JointTrajectoryPoint()

    all_position = [0] * len(joint_names)

    for i, name in enumerate(joint_names):
        all_position[i] = round(joint_position_dict[name], 4)
    rospy.loginfo(all_position)
    point.positions = all_position
    point.time_from_start = rospy.Duration(duration)  # Adjust based on your requirements
    traj_msg.points.append(point)

    # rospy.loginfo(traj_msg)
    
    # Publish the message
    arm_pub.publish(traj_msg)
    # time.sleep(duration)

# Function to get the current pose of the end-effector
def get_current_end_effector_pose():
    current_pose = Frame()
    fk_solver.JntToCart(current_joint_positions, current_pose)
    return current_pose

def update_desired_frame(delta_x=0, delta_y=0, delta_z=0, delta_roll=0, delta_pitch=0, delta_yaw=0):
    global desired_frame
    position, rotation = desired_frame.p, desired_frame.M
    position = Vector(position.x() + delta_x, position.y() + delta_y, position.z() + delta_z)
    # Adjust orientation by converting current rotation to RPY, updating it, and converting back
    roll, pitch, yaw = rotation.GetRPY()
    rotation = Rotation.RPY(roll + delta_roll, pitch + delta_pitch, yaw + delta_yaw)
    desired_frame = Frame(rotation, position)

def on_press(key):
    # Determine the change based on the key pressed
    dis = 0.01
    if key == Key.up:
        update_desired_frame(delta_x=dis)  # Move up along the z-axis
    elif key == Key.down:
        update_desired_frame(delta_x=-dis)  # Move down along the z-axis
    elif key == Key.left:
        update_desired_frame(delta_y=dis)  # Move left along the y-axis
    elif key == Key.right:
        update_desired_frame(delta_y=-dis)  # Move right along the y-axis
    elif hasattr(key, 'char'):
        if key.char == 'w':
            update_desired_frame(delta_z=dis)  # Move up along the z-axis
        elif key.char == 's':
            update_desired_frame(delta_z=-dis)  # Move down along the z-axis
        elif key.char == 'a':
            update_gripper_position(0.015)
        elif key.char == 'd':
            update_gripper_position(-0.015)
        elif key.char == 'q':
            # Rotate arm_7_joint clockwise
            update_desired_frame(delta_yaw=0.1)
        elif key.char == 'e':
            # Rotate arm_7_joint counter-clockwise
            update_desired_frame(delta_yaw=-0.1)  # Adjust this value as needed
        elif key.char == 't':
            update_head_position(tilt_increment=0.2)
        elif key.char == 'g':
            update_head_position(tilt_increment=-0.2)
        elif key.char == 'f':
            update_head_position(pan_increment=0.2)
        elif key.char == 'h':
            update_head_position(pan_increment=-0.2)
    
    # Add more key bindings as needed to control other axes or rotation

def teleop_loop():
    
    # Main loop for teleoperation
    while not rospy.is_shutdown():
        rospy.loginfo("inside the while loop")
        # Update joint velocities based on the current desired_twist
        ik_solver_pos.CartToJnt(current_joint_positions, desired_frame, desired_joint_positions)
        
        # Convert JntArray to list
        # joint_velocities_list = [joint_velocities[i] for i in range(number_of_joints)]
        joint_positions_dict = {joint_names[i]: desired_joint_positions[i] for i in range(number_of_joints)}
        
        # print("dict", joint_positions_dict)
        # Apply the calculated joint positions to the robot
        apply_joint_positions(current_joint_positions, joint_positions_dict)

        rospy.sleep(0.5)  # Adjust the loop rate as needed
    
def run():
    global ik_solver_pos, desired_joint_positions, joint_names, number_of_joints, fk_solver, arm_pub, gripper_client, desired_frame, current_gripper_position, current_joint_positions, current_head_position, head_client
    # Load the robot model from parameter server
    robot_urdf = URDF.from_parameter_server()

    # Generate a KDL tree from the URDF model
    success, kdl_tree = treeFromUrdfModel(robot_urdf)
    if not success:
        rospy.logerr("Failed to extract KDL tree from URDF robot model.")
        exit(1)

    # Specify the chain: from base link to end-effector link
    # base_link = "base_link"
    base_link = "torso_lift_link"
    end_effector_link = "gripper_link"
    chain = kdl_tree.getChain(base_link, end_effector_link)

    # Replace velocity IK solver with position IK solver
    ik_solver_pos = ChainIkSolverPos_LMA(chain)


    # Initialize the joint array with the number of joints
    number_of_joints = chain.getNrOfJoints()
    desired_joint_positions = JntArray(number_of_joints)
    current_joint_positions = JntArray(number_of_joints)
    # rospy.loginfo(number_of_joints)

    '''
    print("Number of Joints in the chain:", chain.getNrOfJoints())
    print("Number of Segments in the chain:", chain.getNrOfSegments())

    # Loop through each segment to print its details
    for i in range(chain.getNrOfSegments()):
        segment = chain.getSegment(i)
        print("Segment", i, ":")
        print("  Name:", segment.getName())
        print("  Joint Name:", segment.getJoint().getName())
        print("  Joint Type:", segment.getJoint().getTypeName())
        print("  Frame to Tip:", segment.getFrameToTip())
        print("  Joint Origin:", segment.getJoint().JointOrigin())
        print("  Joint Axis:", segment.getJoint().JointAxis())

    # If you want to verify the entire structure, you could also print the names of all segments and joints
    print("\nAll segment names in the chain:")
    for i in range(chain.getNrOfSegments()):
        print(chain.getSegment(i).getName())

    print("\nChecking if the base link and end-effector link are correct:")
    print("Base Link:", base_link)
    print("End-effector Link:", end_effector_link)
    '''

    # Initialize Forward Kinematics solver
    fk_solver = ChainFkSolverPos_recursive(chain)

    # List of joint names for TIAGO's arm - Update this list to match your configuration
    joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
                "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
                ]

    current_gripper_position = [0, 0]
    current_head_position = [0, 0]

    # Publisher for controlling the robot's arm
    arm_pub = rospy.Publisher('/arm_controller/safe_command', JointTrajectory, queue_size=1)

    gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()

    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    # Subscribe to the current joint state
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.loginfo("gripper server connected.")

    # wait to get first values
    rospy.wait_for_message("joint_states", JointState)
    rospy.sleep(2.0)
    rospy.loginfo("Get the joint states message")

    # Initialize desired_frame with the current end-effector pose at the start of the script or inside a suitable initialization function
    desired_frame = get_current_end_effector_pose()
    # print("desired frame", desired_frame)SwitchController

    # Start listening to keyboard events
    listener = Listener(on_press=on_press)
    listener.start()
    rospy.loginfo("Listening to key press")

    try:
        teleop_loop()
        # if switch_controllers():
        #     rospy.loginfo("Successfully switched controllers.")
        #     teleop_loop()
        # else:
        #     rospy.loginfo("Failed to switch controllers")
        
    except rospy.ROSInterruptException:
        pass
    finally:
        listener.stop()



if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('tiago_arm_teleop_position')

    run()

    