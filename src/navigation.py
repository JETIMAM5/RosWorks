#!/usr/bin/env python3
"""
navigation.py
Complete navigation system - both goal seeking and obstacle avoidance
"""

import rospy
import math
from geometry_msgs.msg import Twist
from avoider import Avoider


class Navigation:
    def __init__(self, robot, goal_tolerance=0.05, max_linear=0.15, 
                 ang_p=0.6, dist_scale=0.5, front_threshold=0.3):
        self.robot = robot
        self.avoider = Avoider()
        
        # Navigation parameters
        self.GOAL_TOLERANCE = goal_tolerance
        self.MAX_LINEAR = max_linear
        self.ANG_P = ang_p
        self.DIST_LINEAR_SCALE = dist_scale
        self.FRONT_OBS_THRESHOLD = front_threshold
        
        # State
        self.current_goal = None
        self.goal_reached = False
        
    def set_goal(self, x, y):
        """Set new goal"""
        self.current_goal = (x, y)
        self.goal_reached = False
        rospy.loginfo(f"[NAV] New goal set: ({x:.2f}, {y:.2f})")
    
    def has_goal(self):
        """Check if active goal exists"""
        return self.current_goal is not None and not self.goal_reached
    
    def is_goal_reached(self):
        """Check if goal is reached"""
        return self.goal_reached
    
    def sensors_ready(self):
        """Check if sensors are ready"""
        if self.robot.position is None or self.robot.yaw is None:
            rospy.logwarn_throttle(2.0, "[NAV] Waiting for odometry...")
            return False
        if not self.robot.lidar_ranges:
            rospy.logwarn_throttle(2.0, "[NAV] Waiting for lidar...")
            return False
        return True
    
    def compute_navigation_twist(self):
        """Compute twist for goal seeking only"""
        if not self.current_goal:
            return Twist()
            
        gx, gy = self.current_goal
        dx = gx - self.robot.position.x
        dy = gy - self.robot.position.y
        dist = math.hypot(dx, dy)
        
        twist = Twist()
        if dist > self.GOAL_TOLERANCE:
            target_angle = math.atan2(dy, dx)
            angle_error = (target_angle - self.robot.yaw + math.pi) % (2 * math.pi) - math.pi
            
            twist.linear.x = min(self.MAX_LINEAR, dist * self.DIST_LINEAR_SCALE)
            twist.angular.z = self.ANG_P * angle_error
        else:
            rospy.loginfo("[NAV] Goal reached!")
            self.goal_reached = True
            
        return twist
    
    def get_front_distance(self):
        """Get minimum distance ahead"""
        if not self.robot.lidar_ranges:
            return float('inf')
            
        # 60 degree window in front
        half = 30
        front_ranges = self.robot.lidar_ranges[0:half+1] + self.robot.lidar_ranges[-half:]
        filtered = [x for x in front_ranges if x is not None and x > 0 and not math.isinf(x)]
        return min(filtered) if filtered else float('inf')
    
    def needs_obstacle_avoidance(self):
        """Check if obstacle avoidance is needed"""
        return self.get_front_distance() < self.FRONT_OBS_THRESHOLD
    
    def update(self):
        """Main navigation update - single function call"""
        if not self.sensors_ready():
            return Twist()  # Stop
            
        if not self.has_goal():
            return Twist()  # No goal, stop
        
        # Compute navigation twist
        nav_twist = self.compute_navigation_twist()
        
        if self.goal_reached:
            return Twist()  # Reached goal, stop
        
        # Check for obstacles
        if self.needs_obstacle_avoidance():
            avoid_twist = self.avoider.compute_avoidance_twist(self.robot.lidar_ranges)
            rospy.loginfo("[NAV] Obstacle detected - avoiding")
            return avoid_twist
        else:
            return nav_twist
    
    def stop(self):
        """Stop the robot"""
        self.current_goal = None
        self.goal_reached = True
        return Twist()  # Zero twist