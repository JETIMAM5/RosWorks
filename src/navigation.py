#!/usr/bin/env python3
import math
import rospy
from robot import Robot

class Navigation:
    def __init__(self, robot: Robot):
        self.robot = robot

    def goToGoal(self, x, y):
        if self.robot.position is None:
            rospy.logwarn("Position data missing, goToGoal not started!")
            return
        goal_positions = [x, y]
        tolerance = 0.05
        distance = float('inf')
        k_p = 0.6
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and distance > tolerance:
            current_positions = [self.robot.position.x, self.robot.position.y]
            distance = math.sqrt((goal_positions[0]-current_positions[0])**2 + (goal_positions[1]-current_positions[1])**2)
            steering_angle = math.atan2(goal_positions[1]-current_positions[1], goal_positions[0]-current_positions[0])
            yaw_error = math.atan2(math.sin(steering_angle - self.robot.yaw), math.cos(steering_angle - self.robot.yaw))

            self.robot.twist.angular.z = yaw_error * k_p

            if abs(yaw_error) < tolerance:
                self.robot.twist.linear.x = min(0.2, 0.5 * distance)
            else:
                self.robot.twist.linear.x = 0.0

            self.robot.cmd_vel_pub.publish(self.robot.twist)
            rate.sleep()

        self.robot.stop()
        rospy.loginfo("Goal reached!")
