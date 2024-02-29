#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

def sort_assemb_start():
    global height_pub, arm_client, gripper_client
    height_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)
    current_height = rospy.Subscriber("/joint_states", JointState, joint_states_callback)
    
    arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("arm server connected.")
    
    gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()

    rospy.loginfo("gripper server connected.")
    rospy.wait_for_message("joint_states", JointState)
    rospy.sleep(1.0)

    adjust_height(0.34)

    right_joint_angles = [0.07, 0.06, 0.11, 0.04, 1.62, -0.14, 0.00]
    move_arm(right_joint_angles, 6)

    up_joint_angles = [ 1.50, 1.00, 0.11, 1.27, 1.62, -1.25, 0.00]
    move_arm(up_joint_angles, 6)

    rospy.loginfo("finish gesture")
    

def joint_states_callback(msg):
        global current_torso_height
        if "torso_lift_joint" in msg.name:
            index = msg.name.index("torso_lift_joint")
            current_torso_height = msg.position[index]

def adjust_height(target_height):
        rate = rospy.Rate(10)
        duration = 6.0  # Duration for height adjustment

        traj = JointTrajectory()
        traj.joint_names = ["torso_lift_joint"]


        target_height = float(target_height)

        # Initial position
        start_point = JointTrajectoryPoint()
        start_point.positions = [current_torso_height]
        start_point.time_from_start = rospy.Duration(0)
        traj.points.append(start_point)

        # Target position
        target_point = JointTrajectoryPoint()
        target_point.positions = [target_height]
        target_point.time_from_start = rospy.Duration(duration)
        traj.points.append(target_point)

        # Publish trajectory
        height_pub.publish(traj)
        time.sleep(duration)

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
    
    
def move_gripper(width, t):
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    # goal.command.position = width
    trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
    point = JointTrajectoryPoint()
    point.positions = width
    point.time_from_start = rospy.Duration(t)
    trajectory.points.append(point)

    # Set the trajectory in the goal
    goal.trajectory = trajectory
    
    gripper_client.send_goal(goal)
    if gripper_client.wait_for_result():
        rospy.loginfo("Gripper completed successfully.")
    else:
        rospy.loginfo("Gripper did not complete before the timeout.")

if __name__ == '__main__':
    rospy.init_node('start_position')
    sort_assemb_start()
    rospy.loginfo("finish start_position")