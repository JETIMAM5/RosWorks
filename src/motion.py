#!/usr/bin/env python3
import math
import rospy
from robot import Robot

class Motion:
    def __init__(self, robot: Robot):
        self.robot = robot

    def move_straight(self, distance):
        if self.robot.position is None:
            print("Position data is missing!")
            return

        start_x = self.robot.position.x
        start_y = self.robot.position.y
        start_yaw = self.robot.yaw

        self.robot.twist.linear.x = self.robot.linear_velocity
        rate = rospy.Rate(10)
        distance_moved = 0.0
        k_p = 1.0

        while distance_moved < distance and not rospy.is_shutdown():
            yaw_error = math.atan2(math.sin(start_yaw - self.robot.yaw), math.cos(start_yaw - self.robot.yaw))
            self.robot.twist.angular.z = k_p * yaw_error

            current_x = self.robot.position.x
            current_y = self.robot.position.y
            distance_moved = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)

            self.robot.cmd_vel_pub.publish(self.robot.twist)
            rate.sleep()

        self.robot.stop()

    def follow_polygon_trajectory(self, sides, side_length):
        if sides < 3:
            print("Polygon must have at least 3 corners!")
            return

        tolerance = 0.01
        start_yaw = self.robot.yaw
        k_p = 1.0
        angle_increment = 2 * math.pi / sides

        for corner in range(sides):
            self.move_straight(side_length)
            target_yaw = start_yaw + angle_increment * (corner + 1)
            while not rospy.is_shutdown():
                yaw_error = math.atan2(math.sin(target_yaw - self.robot.yaw), math.cos(target_yaw - self.robot.yaw))
                if abs(yaw_error) < tolerance:
                    break
                self.robot.twist.linear.x = 0.0
                self.robot.twist.angular.z = k_p * yaw_error
                self.robot.cmd_vel_pub.publish(self.robot.twist)
                rospy.sleep(0.1)
            self.robot.stop()
            rospy.sleep(0.1)

    def follow_circular_trajectory(self, radius):
        if radius <= 0:
            print("Radius must be positive!")
            return

        angular_speed = self.robot.linear_velocity / radius
        start_yaw = self.robot.yaw
        last_yaw = start_yaw
        angle_turned = 0.0
        rate = rospy.Rate(10)
        k_p = 1.0

        while not rospy.is_shutdown() and angle_turned < 2 * math.pi:
            current_yaw = self.robot.yaw
            delta_yaw = math.atan2(math.sin(current_yaw - last_yaw), math.cos(current_yaw - last_yaw))
            angle_turned += abs(delta_yaw)
            last_yaw = current_yaw

            yaw_error = math.atan2(math.sin(start_yaw + angle_turned - self.robot.yaw), math.cos(start_yaw + angle_turned - self.robot.yaw))
            self.robot.twist.angular.z = angular_speed + k_p * yaw_error
            self.robot.twist.linear.x = self.robot.linear_velocity
            self.robot.cmd_vel_pub.publish(self.robot.twist)
            rate.sleep()

        self.robot.stop()
