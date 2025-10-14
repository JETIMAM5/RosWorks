#!/usr/bin/env python3
"""
navigation.py
Complete navigation system - both goal seeking and obstacle avoidance
"""

import rospy
import math
from geometry_msgs.msg import Twist, Point
from avoider import Avoider
from wall_follower import WallFollower


class Navigation:

    def __init__(self, robot, goal_tolerance=0.05, max_linear=0.15, 
                 ang_p=0.6, dist_scale=0.5, front_threshold=0.3, algorithm="bug0"):
        self.robot = robot
        self.avoider = Avoider()
        self.wall_follower = WallFollower()
        
        # Navigation parameters
        self.GOAL_TOLERANCE = goal_tolerance
        self.MAX_LINEAR = max_linear
        self.ANG_P = ang_p
        self.DIST_LINEAR_SCALE = dist_scale
        self.FRONT_OBS_THRESHOLD = front_threshold

        # Bug0 parameters 
        self.OA_ALGORITHM = algorithm
        self.STATE_GO_TO_GOAL = 0
        self.STATE_AVOID_OBSTACLE = 1
        self.CURRENT_STATE = self.STATE_GO_TO_GOAL

        # Bug1 parameters
        self.STATE_CIRCUMNAVIGATE = 1
        self.STATE_RETURN_TO_CLOSEST = 2
        self.CIRCUMNAVIGATE_STARTING_POINT = None
        self.CIRCUMNAVIGATE_CLOSEST_POINT = None
        self.BUG1_HAS_LEFT_START = False

        # Bug2 parameters
        self.bug2_hit_point = None

        
        # Main states
        self.current_goal = None
        self.goal_reached = False

        rospy.loginfo("[NAV] INITIAL STATE : GO_TO_GOAL")

        
    def set_goal(self, x, y):
    
        # Create a new, empty Point object.
        # A Point object is like a container with .x, .y, and .z attributes.
        goal_point = Point()
        
        # Assign the received x and y values to the attributes of the Point object.
        goal_point.x = x
        goal_point.y = y
        
        # Store the complete Point object as the current goal.
        # Now, self.current_goal is an object, not a simple tuple.
        self.current_goal = goal_point
        
        # Reset the goal_reached flag, as we have a new mission.
        self.goal_reached = False
        
        # Log a confirmation message to the console.
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

        """Computes the Twist message for goal-seeking behavior."""

        if not self.current_goal:
            return Twist()
            
        # Get goal coordinates directly from the Point object's attributes.
        gx = self.current_goal.x
        gy = self.current_goal.y
        
        # Calculate distance and angle to the goal.
        dx = gx - self.robot.position.x
        dy = gy - self.robot.position.y
        dist = math.hypot(dx, dy)
        
        twist = Twist()
        if dist > self.GOAL_TOLERANCE:
            # Calculate the required turn to face the goal.
            target_angle = math.atan2(dy, dx)
            angle_error = (target_angle - self.robot.yaw + math.pi) % (2 * math.pi) - math.pi
            
            # Set linear and angular speeds.
            twist.linear.x = min(self.MAX_LINEAR, dist * self.DIST_LINEAR_SCALE)
            twist.angular.z = self.ANG_P * angle_error
        else:
            # The goal has been reached.
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
    
    
    def calc_dist_points(self, p1, p2):
        """ Calculates the Euclidian distance between two points"""
        if p1 is None or p2 is None:
            return float('inf')
        
        return math.hypot(p1.x - p2.x , p1.y - p2.y)
    
    
    def distance_to_m_line(self):

        """Calculates the perpendicular distance from the robot to the M-Line."""

        if self.bug2_hit_point is None or self.current_goal is None:
            return float('inf')
        
        # P0: Robot's current position.
        p0_x, p0_y = self.robot.position.x, self.robot.position.y
        # P1: Start of the M-Line (the hit point).
        p1_x, p1_y = self.bug2_hit_point.x, self.bug2_hit_point.y
        # P2: End of the M-Line (the goal).
        p2_x, p2_y = self.current_goal.x, self.current_goal.y

        # Standard formula for the distance from a point to a line.
        numerator = abs((p2_y - p1_y) * p0_x - (p2_x - p1_x) * p0_y + p2_x * p1_y - p2_y * p1_x)
        denominator = math.hypot(p2_y - p1_y, p2_x - p1_x)

        # Avoid division by zero.
        if denominator == 0:
            return float('inf')
        
        return numerator / denominator

    
    def update(self):
        
        """Main navigation update - single function call"""
        if not self.sensors_ready():
            return Twist()  # Stop
            
        if not self.has_goal():
            return Twist()  # No goal, stop
        
        if self.get_front_distance() < 0.15 :
            return self.avoider.compute_avoidance_twist(self.robot.lidar_ranges)
        
        if self.OA_ALGORITHM == "bug0" :
            return self.update_bug0()
        
        elif self.OA_ALGORITHM == "bug1":
            return self.update_bug1()
        
        elif self.OA_ALGORITHM == "bug2":
            return self.update_bug2()
        
        else:
            rospy.logerr(f"Unknown algorithm: {self.OA_ALGORITHM}")
            return Twist()
        
        
    # Please replace your entire old update_bug0 function with this new and corrected version.

    def update_bug0(self):
        
        # --- STATE 0: GO TO GOAL ---
        if self.CURRENT_STATE == self.STATE_GO_TO_GOAL:
            if self.needs_obstacle_avoidance():
                rospy.loginfo("[NAV-BUG0] OBSTACLE DETECTED ! SWITCHING THE AVOID_OBSTACLE MODE !")
                self.CURRENT_STATE = self.STATE_AVOID_OBSTACLE
                return self.wall_follower.compute_twist(self.robot.lidar_ranges)

            return self.compute_navigation_twist()
        
        # --- STATE 1: AVOID OBSTACLE ---
        elif self.CURRENT_STATE == self.STATE_AVOID_OBSTACLE:
            
            # --- THE FIX IS HERE ---
            # Instead of unpacking a tuple, we now read the attributes from the Point object.
            gx = self.current_goal.x
            gy = self.current_goal.y
            # --- END OF FIX ---
            
            # Calculate the angle towards the goal to know when to exit this state.
            target_angle = math.atan2(gy - self.robot.position.y, gx - self.robot.position.x)
            angle_error = (target_angle - self.robot.yaw + math.pi ) % (2*math.pi) - math.pi

            # State Transition: If the path to the goal is clear, switch back.
            if abs(angle_error) < (math.pi / 6) and not self.needs_obstacle_avoidance():
                rospy.loginfo("[NAV-BUG0] OBSTACLE BYPASSED ! TURNING BACK TO GO_TO_GOAL MODE !")
                self.CURRENT_STATE = self.STATE_GO_TO_GOAL
                return self.compute_navigation_twist()
        
        # Action: As long as the exit condition is not met, continue following the wall.
        return self.wall_follower.compute_twist(self.robot.lidar_ranges)
    

    # Please replace your entire old update_bug1 function with this new and corrected version.

    def update_bug1(self):
        """
        Executes the Bug1 algorithm logic based on the current state.
        1. GO_TO_GOAL: Moves towards the goal until an obstacle is hit.
        2. CIRCUMNAVIGATE: Follows the obstacle's perimeter to complete a full lap,
        while remembering the point on the perimeter closest to the goal.
        3. RETURN_TO_CLOSEST: Continues following the perimeter until it reaches the
        saved closest point, then switches back to GO_TO_GOAL.
        """

        # --- STATE 0: GO TO GOAL ---
        if self.CURRENT_STATE == self.STATE_GO_TO_GOAL:
            # State Transition: If an obstacle is detected, switch to circumnavigation.
            if self.needs_obstacle_avoidance():
                rospy.loginfo("[NAV-BUG1] Obstacle detected! Switching to CIRCUMNAVIGATE state.")
                self.CURRENT_STATE = self.STATE_CIRCUMNAVIGATE

                # Reset the logic for this new obstacle encounter.
                self.CIRCUMNAVIGATE_STARTING_POINT = self.robot.position
                self.CIRCUMNAVIGATE_CLOSEST_POINT = self.robot.position
                self.bug1_has_left_start = False # Reset the "has left start point" flag.

                # Action: Immediately start following the wall.
                return self.wall_follower.compute_twist(self.robot.lidar_ranges)
        
            # Action: If no obstacle, continue moving towards the goal.
            return self.compute_navigation_twist()
                
        # --- STATE 1: CIRCUMNAVIGATE (and find the closest point) ---
        elif self.CURRENT_STATE == self.STATE_CIRCUMNAVIGATE:

            # Task 1: Continuously update the closest point found so far.
            dist_current_to_goal = self.calc_dist_points(self.robot.position, self.current_goal)
            dist_closest_to_goal = self.calc_dist_points(self.CIRCUMNAVIGATE_CLOSEST_POINT, self.current_goal)

            if dist_current_to_goal < dist_closest_to_goal:
                self.CIRCUMNAVIGATE_CLOSEST_POINT = self.robot.position
                rospy.loginfo("[NAV-BUG1] New closest point to goal identified and saved!")

            # Task 2: Correctly check if a full lap has been completed.
            distance_from_start = self.calc_dist_points(self.robot.position, self.CIRCUMNAVIGATE_STARTING_POINT)

            # First, check if the robot has moved far enough away from the start point.
            # This prevents the algorithm from finishing prematurely.
            if not self.bug1_has_left_start:
                if distance_from_start > 1.0: # Use a significant distance like 1.0 meter
                    rospy.loginfo("[NAV-BUG1] Sufficiently left the starting point. Now monitoring for return.")
                    self.bug1_has_left_start = True
            
            # Only if we have already left the start, check if we have returned.
            else:
                if distance_from_start < 0.4: # Use a reasonable tolerance for returning
                    rospy.loginfo("[NAV-BUG1] Circumnavigation complete! Switching to RETURN_TO_CLOSEST state.")
                    self.CURRENT_STATE = self.STATE_RETURN_TO_CLOSEST

            # Action: Regardless of the tasks above, continue following the wall.
            return self.wall_follower.compute_twist(self.robot.lidar_ranges)
        
        # --- STATE 2: RETURN TO THE CLOSEST POINT ---
        elif self.CURRENT_STATE == self.STATE_RETURN_TO_CLOSEST:

            # State Transition: Check if we have reached the saved closest point.
            if self.calc_dist_points(self.robot.position, self.CIRCUMNAVIGATE_CLOSEST_POINT) < 0.2:
                rospy.loginfo("[NAV-BUG1] Reached the closest point! Switching back to GO_TO_GOAL state.")
                self.CURRENT_STATE = self.STATE_GO_TO_GOAL

                # Clean up memory variables for the next potential obstacle.
                self.CIRCUMNAVIGATE_STARTING_POINT = None
                self.CIRCUMNAVIGATE_CLOSEST_POINT = None
                self.bug1_has_left_start = False # Also reset the flag here

                # Action: Switch back to goal-seeking behavior.
                return self.compute_navigation_twist()

            # Action: Until the closest point is reached, continue following the wall.
            return self.wall_follower.compute_twist(self.robot.lidar_ranges)

    
    def update_bug2(self):
        
        # --- STATE 0: GO TO GOAL ---
        if self.CURRENT_STATE == self.STATE_GO_TO_GOAL:
            if self.needs_obstacle_avoidance():
                rospy.loginfo("[NAV-Bug2] OBSTACLE DETECTED! SWITCHING TO AVOID_OBSTACLE MODE !")
                self.CURRENT_STATE = self.STATE_AVOID_OBSTACLE
                
                # MEMORIZE: Record the current position as the starting point of the M-Line.
                self.bug2_hit_point = self.robot.position
                
                # ACTION: Immediately start following the wall.
                return self.wall_follower.compute_twist(self.robot.lidar_ranges)

            # ACTION: If no obstacle, proceed directly to the goal.
            return self.compute_navigation_twist()

        # --- STATE 1: AVOID OBSTACLE (while searching for the M-Line) ---
        elif self.CURRENT_STATE == self.STATE_AVOID_OBSTACLE:
            
            # State Transition Condition: Have we re-intersected the M-Line?
            dist_to_line = self.distance_to_m_line()
            
            # Check two conditions:
            # 1. Are we close enough to the M-Line (e.g., within 10 cm)?
            # 2. (CRUCIAL!) Is this point on the line closer to the goal than our original hit point?
            #    This prevents getting trapped in U-shaped obstacles.
            is_closer_to_goal = self.calc_dist_points(self.robot.position, self.current_goal) < self.calc_dist_points(self.bug2_hit_point, self.current_goal)

            if dist_to_line < 0.1 and is_closer_to_goal:
                rospy.loginfo("[NAV-BUG2] M-Line RE-ACQUIRED ! SWITCHING BACK TO GO_TO_GOAL MODE.")
                self.CURRENT_STATE = self.STATE_GO_TO_GOAL
                
                # CLEAN MEMORY: The M-Line is no longer needed, reset the hit point for the next obstacle.
                self.bug2_hit_point = None
                
                # ACTION: Switch back to goal-seeking behavior.
                return self.compute_navigation_twist()
            
            # ACTION: Until the M-Line is found, continue following the wall.
            return self.wall_follower.compute_twist(self.robot.lidar_ranges)
        
    
    def stop(self):
        """Stop the robot"""
        self.current_goal = None
        self.goal_reached = True
        return Twist()  # Zero twist