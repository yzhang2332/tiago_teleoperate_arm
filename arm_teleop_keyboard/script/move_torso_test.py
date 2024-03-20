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

def joint_states_callback(msg):
        global current_torso_height
        if "torso_lift_joint" in msg.name:
            index = msg.name.index("torso_lift_joint")
            current_torso_height = msg.position[index]

def sort_assemb_start():
    global height_pub, arm_client, gripper_client
    height_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)
    current_height = rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    rospy.wait_for_message("joint_states", JointState)
    rospy.sleep(1.0)

    adjust_height(0.3)

    # right_joint_angles = [0.07, 0.06, 0.11, 0.04, 1.62, -0.14, 0.00]
    # move_arm(right_joint_angles, 6)

    # up_joint_angles = [0.14, 0.34, -1.38, 1.84, 0.32, -1.33, 0.23]
    # move_arm(up_joint_angles, 6)


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
        rospy.loginfo("torso moving")
        time.sleep(duration)

if __name__ == '__main__':
    rospy.init_node('move_torso_test')
    sort_assemb_start()
    rospy.loginfo("finish torso test")