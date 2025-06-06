#!/usr/bin/env python3

import rospy
import time
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Vector, Rotation, JntArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pynput.keyboard import Key, Listener, KeyCode
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
import numpy as np
import signal
import sys


def signal_handler(sig, frame):
    rospy.loginfo("shutting down...")
    rospy.signal_shutdown("Ctrl+C pressed")
    sys.exit(0)


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

def move_arm(joint_angles, t):
    global doing_action, desired_frame
    doing_action = True
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
    desired_frame = get_current_end_effector_pose()
    doing_action = False

def rotate_gripper(distance_r, t):
    global doing_action, desired_frame
    doing_action = True

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
    joint_angles = [current_joint_positions[i] for i in range(current_joint_positions.rows())]
    joint_angles[-1] += distance_r
    print(joint_angles)
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
    desired_frame = get_current_end_effector_pose()
    doing_action = False

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
    
    
def apply_joint_positions(current_joint_positions, joint_position_dict):
    global duration, doing_action

    # Create a JointTrajectory message
    traj_msg = JointTrajectory()

    # traj_msg.header.stamp = rospy.Time.now()

    traj_msg.joint_names = joint_names


    # start_joint_list = [current_joint_positions[i] for i in range(len(joint_names))]
    # end_joint_list = [joint_position_dict[name] for name in joint_names]

    # num_interp = 4
    # interp_lists = cosine_interp_list(start_joint_list, end_joint_list, num_interp)

    # # Add all interpolated joint lists
    # for i in range(num_interp):
    #     this_joint_list = [interp_lists[joint_index][i] for joint_index in range(len(interp_lists))]

    #     point = JointTrajectoryPoint()
    #     point.positions = this_joint_list
    #     point.time_from_start = rospy.Duration(i * (duration / (num_interp - 1)))  # Adjust based on your requirements
    #     traj_msg.points.append(point)

    
    point = JointTrajectoryPoint()

    all_position = [0] * len(joint_names)

    for i, name in enumerate(joint_names):
        all_position[i] = round(joint_position_dict[name], 4)
    # rospy.loginfo(all_position)
    point.positions = all_position

    point.time_from_start = rospy.Duration(duration)  # Adjust based on your requirements
    
    traj_msg.points.append(point)

    # rospy.loginfo(traj_msg)
    
    # Publish the message
    if doing_action == False:
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
    goal_x = position.x() + delta_x
    goal_y = position.y() + delta_y
    goal_z = position.z() + delta_z
    print(goal_x, goal_y, goal_z)
    if goal_x <= 0.774 and goal_y >= -0.6 and goal_y <= 0.442 and goal_z >= -0.384: # virtual wall
        position = Vector(position.x() + delta_x, position.y() + delta_y, position.z() + delta_z)
        # Adjust orientation by converting current rotation to RPY, updating it, and converting back
        roll, pitch, yaw = rotation.GetRPY()
        rotation = Rotation.RPY(roll + delta_roll, pitch + delta_pitch, yaw + delta_yaw)
        desired_frame = Frame(rotation, position)
        print(position)
    else:
        rospy.loginfo("Can't reach this position!!!")
    

def set_distance(distance, dura):
    global dis, duration
    dis = distance
    duration = dura

def on_press(key):
    global dis
    # print(key)
    # Determine the change based on the key pressed
   
    if key == Key.up:
        update_desired_frame(delta_x=dis)  # Move up along the z-axis
    elif key == Key.down:
        update_desired_frame(delta_x=-dis)  # Move down along the z-axis
    elif key == Key.left:
        update_desired_frame(delta_y=dis)  # Move left along the y-axis
    elif key == Key.right:
        update_desired_frame(delta_y=-dis)  # Move right along the y-axis
    # elif key == Key.alt_r:
    #     rospy.loginfo("Workplace!!")
    #     joint_angles = [0.6288893479007194, 0.5573586512018899, -1.080420348040774, 1.2330983604899601, 0.6671281246748627, -1.2751857028523155, -1.3846061511475216]
    #     move_arm(joint_angles, 4)
    # elif key == Key.alt_l:
    #     rospy.loginfo("Manufacturing reset!!")
    #     joint_angles = [1.2965940220021783, 0.947669900032214, -0.755902407897498, 1.7498113028575375, 0.9982123602409906, -0.908175187301305, -0.9204546205012487]
    #     move_arm(joint_angles, 4)
    elif key.char == '4':
        update_gripper_position(0.03)
    elif key == KeyCode(65437):
        update_gripper_position(-0.03)
    elif hasattr(key, 'char'):
        if key.char == '6':
            update_desired_frame(delta_z=dis)  # Move up along the z-axis
        elif key.char == '3':
            update_desired_frame(delta_z=-dis)  # Move down along the z-axis
        elif key.char == '*':
            # update_desired_frame(delta_yaw=dis*10) 
            rotate_gripper(-1.5, 1)#rotate
        elif key.char == '-':
            # update_desired_frame(delta_yaw=-dis*10)
            rotate_gripper(1.5, 1)#rotate
        elif key.char == '/':
            update_head_position(tilt_increment=0.2)
        elif key.char == '8':
            update_head_position(tilt_increment=-0.2)
        elif key.char == '7':
            update_head_position(pan_increment=0.2)
        elif key.char == '9':
            update_head_position(pan_increment=-0.2)
        elif key.char == '1':
            set_distance(0.01, 0.5)
        elif key.char == "2":
            set_distance(0.05, 0.8)
        elif key.char == "c":
            rospy.loginfo("play card")
            joint_angles = [0.6288893479007194, 0.5573586512018899, -1.080420348040774, 1.2330983604899601, 0.6671281246748627, -1.2751857028523155, -1.3846061511475216]
            move_arm(joint_angles, 4)
        elif key.char == "z":
            rospy.loginfo("discard")
            joint_angles = [1.2965940220021783, 0.947669900032214, -0.755902407897498, 1.7498113028575375, 0.9982123602409906, -0.908175187301305, -0.9204546205012487]
            move_arm(joint_angles, 4)
        elif key.char == "a":
            rospy.loginfo("Row2Position1")
            joint_angles = [0.7074302060209544, 0.9783499227354309, -1.023846386176042, 2.1332655666136926, 1.0510605998680926, -0.7740907961139147, -1.161402650410173]
            move_arm(joint_angles, 4)
        elif key.char == "s":
            rospy.loginfo("Row2Position2")
            joint_angles = [0.5172907653177683, 0.9068194498028808, -1.0051008923043765, 1.9981200666060226, 0.9933584589641552, -0.8951419065827795, -1.3966871779514314]
            move_arm(joint_angles, 4)        
        elif key.char == "d":
            rospy.loginfo("Row2Position3")
            joint_angles = [0.39498485481139456, 0.7632983035972327, -0.9602466991122736, 1.7867193701695072, 0.9282606028323069, -1.008610185782145, -1.647087346911659]
            move_arm(joint_angles, 4)
        elif key.char == "f":
            rospy.loginfo("Row2Position4")
            joint_angles = [0.3427674561705196, 0.6262659821933148, -0.9269588744792833, 1.472049717313964, 0.8694357656788422, -1.1572025544118627, -1.8852582783419667]
            move_arm(joint_angles, 4)
        elif key.char == "g":
            rospy.loginfo("Row2Position5")
            joint_angles = [0.3272587046940434, 0.48369591669146633, -0.9087042609708693, 1.1437888144008959, 0.7789012436954862, -1.2156883595295793, -2.074514112286669]
            move_arm(joint_angles, 4)
        elif key.char == "q":
            rospy.loginfo("Row1Position1")
            joint_angles = [0.7220338968276856, 0.5641082561965975, -1.4035116671283498, 1.8524359787997975, 0.523471173221286, -1.2154308433931365, -0.685625840942472]
            move_arm(joint_angles, 4)
        elif key.char == "w":
            rospy.loginfo("Row1Position2")
            joint_angles = [0.4784651965868474, 0.4941731444446149, -1.3934946397157497, 1.7831911675586372, 0.48925302185367153, -1.2496415842244504, -0.9654588074542586]
            move_arm(joint_angles, 4)
        elif key.char == "e":
            rospy.loginfo("Row1Position3")
            joint_angles = [0.3708243369326112, 0.392867709478593, -1.3638577377844423, 1.4877885689607142, 0.47303135789185063, -1.3568627813224843, -1.3359948856502375]
            move_arm(joint_angles, 4)
        elif key.char == "r":
            rospy.loginfo("Row1Position4")
            joint_angles = [0.45422797865130615, 0.5136702988725091, -0.971352867330838, 1.1204719971464512, 0.7982649750487243, -1.3125940900973003, -1.75216875165292]
            move_arm(joint_angles, 4)
        elif key.char == "t":
            rospy.loginfo("Row1Position5")
            joint_angles = [0.5186100062940067, 0.2818060272929482, -0.8810002004698645, 0.5369686453539708, 0.8088990946399591, -1.3936727677612029, -2.0744455648258895]
            move_arm(joint_angles, 4)
        
        # elif key.char == "3":
        #     set_distance(0.10, 0.8)
        # elif key.char == "4":
        #     set_distance(0.12, 1.0)
        # elif key.char == "5":
        #     set_distance(0.15, 1.2)

    # Add more key bindings as needed to control other axes or rotation

def publish_frame():
    # Create a PoseStamped message
    position, rotation = desired_frame.p, desired_frame.M
    position = Vector(position.x(), position.y(), position.z())
    # Adjust orientation by converting current rotation to RPY, updating it, and converting back
    roll, pitch, yaw = rotation.GetRPY()
    q = quaternion_from_euler(roll, pitch, yaw)
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "torso_lift_link"  # Set the reference frame
    pose_msg.pose.position = Point(position.x(), position.y(), position.z())
    pose_msg.pose.orientation = Quaternion(*q)
    # Publish the PoseStamped message
    frame_pub.publish(pose_msg)
    # rospy.loginfo("publish desired frame")

def teleop_loop():
    global dis, duration
    # Main loop for teleoperation
    while not rospy.is_shutdown():
        # rospy.loginfo("inside the while loop")
        rospy.loginfo(f"{dis}, {duration}")
        # Update joint velocities based on the current desired_twist
        ik_solver_pos.CartToJnt(current_joint_positions, desired_frame, desired_joint_positions)
        
        # Convert JntArray to list
        # joint_velocities_list = [joint_velocities[i] for i in range(number_of_joints)]
        joint_positions_dict = {joint_names[i]: desired_joint_positions[i] for i in range(number_of_joints)}
        
        # print("dict", joint_positions_dict)
        # Apply the calculated joint positions to the robot
        apply_joint_positions(current_joint_positions, joint_positions_dict)

        publish_frame()       

        rospy.sleep(0.1)  # Adjust the loop rate as needed
    
def run():
    global ik_solver_pos, desired_joint_positions, joint_names, number_of_joints, fk_solver, arm_pub, frame_pub, gripper_client, arm_client, desired_frame, current_gripper_position, current_joint_positions, current_head_position, head_client
    global dis, duration, doing_action
    doing_action=False
    dis = 0.01
    duration = 0.5
    
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
    end_effector_link = "gripper_grasping_frame_Z"
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
    frame_pub = rospy.Publisher('/arm_cartesian', PoseStamped, queue_size=1)

    gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()

    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("arm server connected.")

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
        
    except rospy.ROSInterruptException:
        pass
    finally:
        listener.stop()



if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('tiago_arm_teleop_position')
    signal.signal(signal.SIGINT, signal_handler)
    while not rospy.is_shutdown():
        try:
            run()
        except rospy.ROSInterruptException:
            pass
    # run()


    