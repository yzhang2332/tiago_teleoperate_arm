#!/usr/bin/env python3

import rospy
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import ChainIkSolverVel_pinv, Twist, Vector, Rotation, JntArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pynput.keyboard import Key, Listener

# Initialize ROS node
rospy.init_node('tiago_arm_teleop')

# Load the robot model from parameter server
# robot_urdf = URDF.load('/home/yanzhang/HRI_competition_2024/tiagocomp_ws/src/tiago_robot/tiago_description/robots/tiago.urdf')
robot_urdf = URDF.from_parameter_server()
# def load_urdf_file(file_path):
#     with open(file_path, 'r') as file:
#         urdf_string = file.read()
#     return URDF.from_xml_string(urdf_string)

# # Load the robot model
# robot_urdf = load_urdf_file('/home/yanzhang/HRI_competition_2024/tiagocomp_ws/src/tiago_robot/tiago_description/robots/tiago.urdf')

# Generate a KDL tree from the URDF model
success, kdl_tree = treeFromUrdfModel(robot_urdf)
if not success:
    rospy.logerr("Failed to extract KDL tree from URDF robot model.")
    exit(1)

# Specify the chain: from base link to end-effector link
base_link = "base_link"  # You might need to adjust this according to TIAGO's URDF
# end_effector_link = "hand_tool_link"  # Adjust this to the actual end-effector link name
# end_effector_link = "hand_palm_link"
end_effector_link = "arm_tool_link"
chain = kdl_tree.getChain(base_link, end_effector_link)

# Create the velocity IK solver
ik_solver_vel = ChainIkSolverVel_pinv(chain)

# Desired end-effector velocity: linear velocity along x, and angular velocity around z
desired_twist = Twist(Vector(0.1, 0, 0), Vector(0, 0, 0.1))

# Initialize the joint array with the number of joints
number_of_joints = chain.getNrOfJoints()
joint_velocities = JntArray(number_of_joints)
current_joint_positions = JntArray(number_of_joints)

# Publisher for controlling the robot's arm
arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

# List of joint names for TIAGO's arm - Update this list to match your configuration
# joint_names = ["torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint", 
#                "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint", 
#                ]

joint_names = ["torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint", 
               "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint", 
               "gripper_left_finger_joint", "gripper_right_finger_joint",
               "head_1_joint", "head_2_joint", 
               "wheel_left_joint", "wheel_right_joint",
               "caster_back_left_1_joint", "caster_back_left_2_joint", 
               "caster_front_left_1_joint", "caster_front_left_2_joint", 
               "caster_back_right_1_joint", "caster_back_right_2_joint",
               "caster_front_right_1_joint", "caster_front_right_2_joint",
               "suspension_left_joint", "suspension_right_joint"]

predict_joint_names = ["torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint", 
               "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
               ]

arm_joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
               "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
               ]



# Calculate joint velocities
# ik_solver_vel.CartToJnt(current_joint_positions, desired_twist, joint_velocities)

# Subscriber callback for updating current joint positions
def joint_state_callback(msg):
    global current_joint_positions
    try:
        # Iterate over the joint names we're interested in, and update their positions
        # for i, name in enumerate(joint_names):
        #     if name in msg.name:
        #         index = msg.name.index(name)
        #         current_joint_positions[i] = msg.position[index]
        #     else:
        #         rospy.logwarn(f"Joint {name} not found in the JointState message.")

        for i, name in enumerate(predict_joint_names):
            index = msg.name.index(name)
            current_joint_positions[i] = msg.position[index]

            # else:
            #     rospy.logwarn(f"Joint {name} not found in the JointState message.")

    except Exception as e:
        rospy.logerr(f"Error in joint_state_callback: {e}")
 

# Subscribe to the current joint state
rospy.Subscriber('/joint_states', JointState, joint_state_callback)

def apply_gripper():
    gripper = [0,0]
    return gripper

def apply_joint_velocities(joint_names, joint_velocities_dict):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names
    
    point = JointTrajectoryPoint()

    all_velocities = [0] * len(joint_names)

    for i, name in enumerate(joint_names):
        if name in joint_velocities_dict:
            # replace the velocity in all_velocity list
            all_velocities[i] = joint_velocities_dict[name]
    
    point.velocities = all_velocities
    point.time_from_start = rospy.Duration(1)  # Adjust based on your requirements
    traj_msg.points.append(point)
    
    # Publish the message
    arm_pub.publish(traj_msg)


desired_twist = Twist()

def on_press(key):
    global desired_twist
    # Update desired_twist based on keyboard input
    if key == Key.up:
        desired_twist = Twist(Vector(0.1, 0, 0), Vector(0, 0, 0))
    elif key == Key.down:
        desired_twist = Twist(Vector(-0.1, 0, 0), Vector(0, 0, 0))
    elif key == Key.left:
        desired_twist = Twist(Vector(0, 0.1, 0), Vector(0, 0, 0))
    elif key == Key.right:
        desired_twist = Twist(Vector(0, -0.1, 0), Vector(0, 0, 0))
    # Add more conditions for other keys if needed

def on_release(key):
    global desired_twist
    # Stop the movement when the key is released
    if key in [Key.up, Key.down, Key.left, Key.right]:
        desired_twist = Twist()

def teleop_loop():
    # Main loop for teleoperation
    while not rospy.is_shutdown():
        # Update joint velocities based on the current desired_twist
        ik_solver_vel.CartToJnt(current_joint_positions, desired_twist, joint_velocities)
        
        # Convert JntArray to list
        # joint_velocities_list = [joint_velocities[i] for i in range(number_of_joints)]
        joint_velocities_dict = {joint_names[i]: joint_velocities[i] for i in range(number_of_joints)}
        # rospy.loginfo(joint_velocities_list)

        # Apply the calculated joint velocities to the robot
        apply_joint_velocities(joint_names, joint_velocities_dict)

        rospy.sleep(0.1)  # Adjust the loop rate as needed

if __name__ == "__main__":
    # Start listening to keyboard events
    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        teleop_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        listener.stop()