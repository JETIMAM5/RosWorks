#!/usr/bin/env python3
"""
run_robot.py
Main, automated entry point for the navigation system.
This script is intended to be launched by a ROS launch file.
"""

import rospy
from robot import Robot
from navigation import Navigation


def main():
    # Initialize the ROS node. The name is given by the launch file.
    rospy.init_node('robot_navigation_node')

    # Create instances of our core classes
    robot = Robot()

    # --- Read Parameters from the Launch File ---
    # Get the private parameters passed from the launch file.
    # The '~' means the parameter is private to this node.
    # If the parameter isn't specified in the launch file, use the default value.
    goal_x = rospy.get_param("~target_x", 3.0)
    goal_y = rospy.get_param("~target_y", 0.0)
    selected_algorithm = rospy.get_param("~nav_algorithm", "bug2")

    # Initialize the Navigation brain with the selected algorithm.
    navigation = Navigation(robot, algorithm=selected_algorithm)

    rospy.sleep(1.0)  # A short wait to ensure everything is initialized

    # Set the goal once at the beginning.
    navigation.set_goal(goal_x, goal_y)

    rate = rospy.Rate(10)  # 10 Hz

    try:
        # --- Main Control Loop ---
        # Keep running until ROS is shut down or the goal is reached.
        while not rospy.is_shutdown():

            # Check if the mission is complete.
            if navigation.is_goal_reached():
                rospy.loginfo("Goal has been reached successfully! Mission complete.")
                break  # Exit the loop to end the program.

            # If the mission is not complete, calculate and publish the next move.
            twist = navigation.update()
            robot.cmd_vel_pub.publish(twist)

            rate.sleep()

    except rospy.ROSInterruptException:
        # Catch the exception thrown when Ctrl+C is pressed.
        print("Program interrupted by user.")
    finally:
        # In any case (mission success or interruption), stop the robot.
        stop_twist = navigation.stop()
        robot.cmd_vel_pub.publish(stop_twist)
        print("Robot has been stopped.")


if __name__ == "__main__":
    main()