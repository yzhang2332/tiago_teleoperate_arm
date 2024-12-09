#! /usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class ArmController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('run_traj_control')
        

        # Create the action client
        self.client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()
        
        # Log that the client has been created
        rospy.sleep(1.0)
        rospy.loginfo("Arm control client has been created.")
    
    def create_goal(self):
        # Create a trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        
        # Define two waypoints
        point1 = JointTrajectoryPoint()
        point1.positions = [0.2, 0.0, -1.5, 1.94, -1.57, -0.5, 0.0]
        point1.time_from_start = rospy.Duration(2.0)
        
        point2 = JointTrajectoryPoint()
        point2.positions = [2.5, 0.2, -2.1, 1.9, 1.0, -0.5, 0.0]
        point2.time_from_start = rospy.Duration(4.0)
        
        goal.trajectory.points.append(point1)
        goal.trajectory.points.append(point2)
        print(goal)
        return goal
    
    def send_goal(self, goal):
        # Set the trajectory start time
       # goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
        
        # Send the goal
        self.client.send_goal(goal)
        rospy.loginfo("Goal sent, waiting for execution to complete.")
        
        # Wait for result
        if self.client.wait_for_result(10):
            rospy.loginfo("finished")
        else:
            rospy.loginfo("failed")
        
        rospy.loginfo("Action finished: %s", self.client.get_state())
        
# Main execution logic
if __name__ == '__main__':
    try:
        arm_controller = ArmController()
        arm_goal = arm_controller.create_goal()
        arm_controller.send_goal(arm_goal)
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")


