#!/usr/bin/env python3

import rospy
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Vector, Rotation, JntArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pynput.keyboard import Key, Listener

# Initialize ROS node
rospy.init_node('tiago_arm_teleop_position')

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
print(number_of_joints)

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

# Publisher for controlling the robot's arm
arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

# List of joint names for TIAGO's arm - Update this list to match your configuration
joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
               "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
               ]

arm_joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
               "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
               ]

# Subscriber callback for updating current joint positions
def joint_state_callback(msg):
    global current_joint_positions
    try:
        for i, name in enumerate(joint_names):
            index = msg.name.index(name)
            current_joint_positions[i] = msg.position[index]
    except Exception as e:
        rospy.logerr(f"Error in joint_state_callback: {e}")
    # print("current postion", current_joint_positions)
 

# Subscribe to the current joint state
rospy.Subscriber('/joint_states', JointState, joint_state_callback)

# wait to get first values
rospy.sleep(1.0)


def apply_joint_positions(joint_position_dict):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = arm_joint_names
    
    point = JointTrajectoryPoint()

    all_velocities = [0] * len(arm_joint_names)

    for i, name in enumerate(arm_joint_names):
        all_velocities[i] = joint_position_dict[name]
    
    point.positions = all_velocities
    point.time_from_start = rospy.Duration(1)  # Adjust based on your requirements
    traj_msg.points.append(point)

    # print("message", traj_msg)
    
    # Publish the message
    arm_pub.publish(traj_msg)


# Initialize Forward Kinematics solver
fk_solver = ChainFkSolverPos_recursive(chain)

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
    if key == Key.up:
        update_desired_frame(delta_x=0.05)  # Move up along the z-axis
    elif key == Key.down:
        update_desired_frame(delta_x=-0.05)  # Move down along the z-axis
    elif key == Key.left:
        update_desired_frame(delta_y=0.05)  # Move left along the y-axis
    elif key == Key.right:
        update_desired_frame(delta_y=-0.05)  # Move right along the y-axis
    # Add more key bindings as needed to control other axes or rotation

# Initialize desired_frame with the current end-effector pose at the start of the script or inside a suitable initialization function
desired_frame = get_current_end_effector_pose()
print("desired frame", desired_frame)

def teleop_loop():
    # Main loop for teleoperation
    while not rospy.is_shutdown():
        # Update joint velocities based on the current desired_twist
        ik_solver_pos.CartToJnt(current_joint_positions, desired_frame, desired_joint_positions)
        
        # Convert JntArray to list
        # joint_velocities_list = [joint_velocities[i] for i in range(number_of_joints)]
        joint_positions_dict = {joint_names[i]: desired_joint_positions[i] for i in range(number_of_joints)}
        
        # print("dict", joint_positions_dict)
        # Apply the calculated joint positions to the robot
        apply_joint_positions(joint_positions_dict)

        rospy.sleep(0.1)  # Adjust the loop rate as needed

if __name__ == "__main__":
    # Start listening to keyboard events
    listener = Listener(on_press=on_press)
    listener.start()

    try:
        teleop_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        listener.stop()