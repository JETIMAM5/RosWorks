#!/usr/bin/env python3
"""
avoider.py
Clean obstacle avoidance module focused solely on avoiding obstacles.
The algorithm to be implemented here is in BUG0 logic.
"""

import rospy
import math
from geometry_msgs.msg import Twist


class Avoider:
    def __init__(self, obstacle_threshold=0.5, regional_angle=30,
                 normal_lin_vel=0.5, trans_lin_vel=-0.09, trans_ang_vel=1.75):
        
        self.OBSTACLE_DIST = obstacle_threshold
        self.REGIONAL_ANGLE = regional_angle
        self.NORMAL_LIN_VEL = normal_lin_vel
        self.TRANS_LIN_VEL = trans_lin_vel
        self.TRANS_ANG_VEL = trans_ang_vel
        
        # Region names and positions
        self.region_names = [
            "front_C", "front_L", "left_R", "left_C", "left_L", "back_R",
            "back_C", "back_L", "right_R", "right_C", "right_L", "front_R"
        ]
        
        self.region_positions = {
            "front_C": 0, "front_L": 1, "left_R": 2, "left_C": 3, 
            "left_L": 4, "back_R": 5, "back_C": 6, "back_L": -5, 
            "right_R": -4, "right_C": -3, "right_L": -2, "front_R": -1
        }
        
        self.regions = {}
    
    def analyze_regions(self, lidar_ranges):
        """Divide lidar data into regions"""
        if not lidar_ranges:
            return
            
        self.regions = {}
        
        # Front center region (special processing)
        half = int(self.REGIONAL_ANGLE / 2)
        front_center_ranges = lidar_ranges[:half+1] + lidar_ranges[-half:]
        self.regions["front_C"] = [x for x in front_center_ranges 
                                   if x <= self.OBSTACLE_DIST and x != float('inf') and x > 0]
        
        # Other regions
        for i, region_name in enumerate(self.region_names[1:]):
            start_idx = self.REGIONAL_ANGLE * i
            end_idx = self.REGIONAL_ANGLE * (i + 1)
            region_ranges = lidar_ranges[start_idx:end_idx]
            
            self.regions[region_name] = [x for x in region_ranges 
                                         if x <= self.OBSTACLE_DIST and x != float('inf') and x > 0]
    
    def find_best_direction(self):
        """Find the best avoidance direction"""
        goal_region = "front_C"
        closest_distance = 1e6
        best_choice = {"region": "back_C", "distance": 1e-6}
        
        for region_name, obstacles in self.regions.items():
            region_distance = abs(self.region_positions[region_name] - 
                                self.region_positions[goal_region])
            
            if not obstacles:  # No obstacles in region
                if region_distance < closest_distance:
                    closest_distance = region_distance
                    best_choice["distance"] = self.OBSTACLE_DIST
                    best_choice["region"] = region_name
            elif max(obstacles) > best_choice["distance"]:  # Farther obstacle
                best_choice["distance"] = max(obstacles)
                best_choice["region"] = region_name
        
        return best_choice
    
    def compute_avoidance_twist(self, lidar_ranges):
        """Main function: compute obstacle avoidance twist"""
        if not lidar_ranges:
            return Twist()
        
        # Analyze regions
        self.analyze_regions(lidar_ranges)
        
        # Find best direction
        best_direction = self.find_best_direction()
        
        # Calculate twist
        twist = Twist()
        
        # Calculate direction difference
        direction_diff = (self.region_positions[best_direction["region"]] - 
                         self.region_positions["front_C"])
        
        if direction_diff == 0:  # Front center clear
            twist.linear.x = self.NORMAL_LIN_VEL
            twist.angular.z = 0
        else:  # Avoidance required
            twist.linear.x = self.TRANS_LIN_VEL
            # Normalize direction and set angular velocity
            twist.angular.z = (direction_diff / abs(direction_diff)) * self.TRANS_ANG_VEL
        
        # Zero other axes
        twist.linear.y = twist.linear.z = 0
        twist.angular.x = twist.angular.y = 0
        
        return twist
    
    def is_obstacle_ahead(self, lidar_ranges, threshold_distance=0.3, angle_window=60):
        """Check if obstacle ahead (for external use)"""
        if not lidar_ranges:
            return False
            
        half = angle_window // 2
        front_ranges = lidar_ranges[0:half+1] + lidar_ranges[-half:]
        filtered = [x for x in front_ranges if x is not None and x > 0 and not math.isinf(x)]
        
        if not filtered:
            return False
            
        return min(filtered) < threshold_distance