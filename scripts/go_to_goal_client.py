#!/usr/bin/env python3

import rospy
import actionlib
from turtlebot3_trajectory_pkg.msg import GoToGoalAction, GoToGoalGoal

def go_to_goal_client(x, y):
    client = actionlib.SimpleActionClient('go_to_goal', GoToGoalAction)
    rospy.loginfo("Waiting for GoToGoal action server...")
    client.wait_for_server()
    rospy.loginfo("Server connected!")

    goal = GoToGoalGoal()
    goal.x = x
    goal.y = y

    rospy.loginfo(f"Sending goal: x={x}, y={y}")
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo(f"Result: {result}")
    return result

if __name__ == "__main__":
    try:
        rospy.init_node('go_to_goal_client_node')
        x = float(input("Target x: "))
        y = float(input("Target y: "))
        go_to_goal_client(x, y)
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
