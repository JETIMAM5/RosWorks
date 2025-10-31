#!/usr/bin/env python3

""" Wall Following System For Bug1"""

import rospy
import math
from geometry_msgs.msg import Twist 

class WallFollower:

    """This class enables the robot to follow an obstacle's boundary, circumnavigate it, and detect important points such as hit and leave positions."""

    def __init__(self, desired_distance = 0.3, proportioan_gain = 1.5, forward_speed = 0.15, front_threshold = 0.4):

        self.DESIRED_DISTANCE = desired_distance
        self.PROPORTIONAL_GAIN = proportioan_gain
        self.FORWARD_SPEED = forward_speed
        self.FRONT_THRESHOLD = front_threshold


    def get_relevant_distances(self, lidar_ranges):
        """
        Gets the lidar ranges for following the wall of an obstacle.
        This version is robust against '0.0', 'inf', and 'NaN' values
        and uses windows for stability.
        """

        if not lidar_ranges:
            return float('inf'), float('inf')

        # --- Robust Right Distance ---
        # Get a 10-degree window around the 270-degree mark (right side)
        right_window = lidar_ranges[265:276]
        
        # Filter out invalid readings (e.g., 0.0, inf, nan)
        filtered_right = [r for r in right_window if r is not None and r > 0.01 and not math.isinf(r) and not math.isnan(r)]
        
        # Take the minimum valid distance from the window
        right_distance = min(filtered_right) if filtered_right else float('inf')

        # --- Robust Front Distance ---
        # Get a 60-degree window at the front (30 degrees left, 30 degrees right)
        half_window = 30
        front_window = lidar_ranges[0:half_window + 1] + lidar_ranges[-half_window:]
        
        # Filter out invalid readings (e.g., 0.0, inf, nan)
        filtered_front = [r for r in front_window if r is not None and r > 0.01 and not math.isinf(r) and not math.isnan(r)]
        
        # Take the minimum valid distance from the window
        front_distance = min(filtered_front) if filtered_front else float('inf')

        return right_distance, front_distance

    
    def compute_twist(self, lidar_ranges):
        
        """ Computes the twist message to follow an obstacle's wall. This is the main function of this class"""
        
        right_dist, front_dist = self.get_relevant_distances(lidar_ranges)

        twist = Twist()


        # 1. State : Emergency Situation ! There is a wall ahead !
        # Action   : Leave everything and turn left
        if front_dist < self.FRONT_THRESHOLD:
            
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            rospy.logwarn_throttle(1, "[WALL_FOLLOWER] OBSTACLE AHEAD ! TURNING LEFT")
            return twist
        
        # 2. State : Wall lost or too far ahead
        # Action   : Go forward-right to reqacquire the wall
        if math.isinf(right_dist) or right_dist > self.DESIRED_DISTANCE * 2:
            
            twist.linear.x = self.FORWARD_SPEED * 0.5
            twist.angular.z = -0.3
            rospy.logwarn_throttle(1, "[WALL_FOLLOWER] WALL CANNOT FOUND , SEARCHING ...")
            return twist
        
        # 3. State : Main Wall-following logic with P-Control
        # Action   : Calculate the error rate (how far the robot is from the wall?)
        error = self.DESIRED_DISTANCE - right_dist
        twist.angular.z = self.PROPORTIONAL_GAIN * error
        twist.linear.x = self.FORWARD_SPEED

        return twist




        
        


