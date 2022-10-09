#!/usr/bin/env python
import math
import argparse
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult, \
    FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
import moveit_msgs.msg
import numpy as np


def jog(joint_positions, action_client):

    startingPoint = JointTrajectoryPoint()
    startingPoint.positions = arm_group.get_current_joint_values()
    startingPoint.time_from_start.secs = 0

    trajectory = JointTrajectory()
    trajectory.points = [startingPoint]

    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration.from_sec(1.0)
    trajectory.points.append(point)

    trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory

    action_client.send_goal(goal)
    action_client.wait_for_result()
    print(f"action_client.result={action_client.get_result()}")


if __name__ == '__main__':
    # parsing input arguments
    parser = argparse.ArgumentParser(description='Jogging')
    parser.add_argument("j1")
    parser.add_argument("j2")
    parser.add_argument("j3")
    parser.add_argument("j4")
    parser.add_argument("j5")
    parser.add_argument("j6")
    parser.add_argument("j7")
    args = parser.parse_args()

    j1 = float(args.j1) * math.pi / 180.0
    j2 = float(args.j2) * math.pi / 180.0
    j3 = float(args.j3) * math.pi / 180.0
    j4 = float(args.j4) * math.pi / 180.0
    j5 = float(args.j5) * math.pi / 180.0
    j6 = float(args.j6) * math.pi / 180.0
    j7 = float(args.j7) * math.pi / 180.0

    print(f"j1={args.j1}, j2={args.j2}, j3={args.j3}, j4={args.j4}, j5={args.j5}, j6={args.j6}, j7={args.j7}")
    print(f"Rad: j1={j1}, j2={j2}, j3={j3}, j4={j4}, j5={j5}, j6={j6}, j7={j7}")

    moveit_commander.roscpp_initialize('')
    rospy.init_node('robot_jogging')

    client = actionlib.SimpleActionClient('/xarm/xarm7_traj_controller/follow_joint_trajectory',
                                          FollowJointTrajectoryAction)
    client.wait_for_server()

    arm_group_name = "xarm7"
    arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
    jog([j1, j2, j3, j4, j5, j6, j7], client)

