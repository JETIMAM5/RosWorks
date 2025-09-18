#!/usr/bin/env python3

# import rospy
# import math
# import time

# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion


# class Robot:
#     def __init__(self):
#         # Base variables
#         self.twist = Twist()
#         self.position = None
#         self.orientation = None
#         self.yaw = None
#         self.odom = None
#         self.running = True

#         # Velocities
#         self.linear_velocity = 0.2
#         self.angular_velocity = 0.5

#         # ROS Publisher and Subscriber
#         self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
#         self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

#     # Getting Odometry Data
#     def get_initial_position(self, timeout=5):
#         start_time = time.time()
#         while self.odom is None:
#             if time.time() - start_time > timeout:
#                 print("Odometry data not received!")
#                 return None
#             rospy.sleep(0.1)
#         return self.odom

#     # Odometry callback
#     def odom_callback(self, msg):
#         self.odom = msg
#         self.position = msg.pose.pose.position
#         self.orientation = msg.pose.pose.orientation
#         q = (
#             self.orientation.x,
#             self.orientation.y,
#             self.orientation.z,
#             self.orientation.w
#         )
#         self.yaw = euler_from_quaternion(q)[2]  # Yaw around Z axis

#     # Move straight while keeping correct heading
#     def move_straight(self, distance):
#         if self.position is None:
#             print("Position data is missing!")
#             return

#         start_x = self.position.x
#         start_y = self.position.y
#         start_yaw = self.yaw

#         self.twist.linear.x = self.linear_velocity
#         rate = rospy.Rate(10)
#         distance_moved = 0.0
#         k_p = 1.0

#         while distance_moved < distance and not rospy.is_shutdown():
#             # Normalize yaw error between ±π
#             yaw_error = math.atan2(math.sin(start_yaw - self.yaw), math.cos(start_yaw - self.yaw))
#             self.twist.angular.z = k_p * yaw_error

#             current_x = self.position.x
#             current_y = self.position.y
#             distance_moved = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)

#             self.cmd_vel_pub.publish(self.twist)
#             rate.sleep()

#         self.stop()

#     # Stop the robot
#     def stop(self):
#         self.twist.linear.x = 0.0
#         self.twist.angular.z = 0.0
#         self.cmd_vel_pub.publish(self.twist)
#         rospy.sleep(0.1)
    

#     # Polygon-based trajectory (square, triangle, etc.)
#     def follow_polygon_trajectory(self, sides, side_length):
#         if sides < 3:
#             print("Polygon must have at least 3 corners!")
#             return

#         tolerance = 0.01
#         start_yaw = self.yaw
#         k_p = 1.0
#         angle_increment = 2 * math.pi / sides  # Angle to turn at each corner

#         for corner in range(sides):
#             self.move_straight(side_length)

#             target_yaw = start_yaw + angle_increment * (corner + 1)
#             while not rospy.is_shutdown():
#                 yaw_error = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
#                 if abs(yaw_error) < tolerance:
#                     break
#                 self.twist.linear.x = 0.0
#                 self.twist.angular.z = k_p * yaw_error
#                 self.cmd_vel_pub.publish(self.twist)
#                 rospy.sleep(0.1)

#             self.stop()
#             rospy.sleep(0.1)

#     # Draw a circle
#     def follow_circular_trajectory(self, radius):
#         if radius <= 0:
#             print("Radius must be positive!")
#             return

#         angular_speed = self.linear_velocity / radius  # Base angular speed
#         start_yaw = self.yaw
#         last_yaw = start_yaw
#         angle_turned = 0.0
#         rate = rospy.Rate(10)
#         k_p = 1.0  # P-controller

#         while not rospy.is_shutdown() and angle_turned < 2 * math.pi:
#             current_yaw = self.yaw
#             # Normalize
#             delta_yaw = math.atan2(math.sin(current_yaw - last_yaw), math.cos(current_yaw - last_yaw))
#             angle_turned += abs(delta_yaw)
#             last_yaw = current_yaw

#             # Angular speed = base angular speed + P-control correction
#             yaw_error = math.atan2(math.sin(start_yaw + angle_turned - self.yaw), math.cos(start_yaw + angle_turned - self.yaw))
#             self.twist.angular.z = angular_speed + k_p * yaw_error
#             self.twist.linear.x = self.linear_velocity
#             self.cmd_vel_pub.publish(self.twist)

#             rate.sleep()

#         self.stop()

#     # Move to a specific goal coordinate
#     def goToGoal(self, x, y):
#         if self.position is None:
#             rospy.logwarn("Position data missing, goToGoal not started!")
#             return
#         goal_positions = [x, y]
#         tolerance = 0.05
#         distance = float('inf')
#         k_p = 0.6
#         rate = rospy.Rate(10)

#         while (not rospy.is_shutdown() and distance > tolerance):
#             current_positions = [self.position.x, self.position.y]
#             distance =  math.sqrt((goal_positions[0] - current_positions[0])**2 + (goal_positions[1] - current_positions[1])**2)
#             steering_angle = math.atan2(goal_positions[1]- current_positions[1], goal_positions[0]- current_positions[0])
#             yaw_error = steering_angle - self.yaw
#             yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

#             self.twist.angular.z = yaw_error * k_p

#             if abs(yaw_error) < tolerance:
#                 self.twist.linear.x = min(0.2, 0.5 * distance)
#             else:
#                 self.twist.linear.x = 0.0

#             self.cmd_vel_pub.publish(self.twist)
#             rate.sleep()

#         self.stop()
#         rospy.loginfo("Goal reached!")
        

# if __name__ == "__main__":

#     # ------------------------ VARIOUS TRAJECTORY EXAMPLES -------------------------------------
#     # Example usage:
#     #my_robot.follow_polygon_trajectory(sides=4, side_length=3)      # Square
#     #my_robot.follow_polygon_trajectory(sides=3, side_length=2)    # Triangle
#     #my_robot.follow_circular_trajectory(radius=1.5)                # Circle
#     # --------------------------------------------------------------------------------------

#     # ----------------------- COMMON NODE START ----------------------------------------
#     rospy.init_node("robot_controller", anonymous=True)
#     my_robot = Robot()

#     # Check odometry
#     if my_robot.get_initial_position() is None:
#         print("Cannot start, odometry not available!")
#         exit(1)

#     rospy.sleep(1)  # Wait for subscribers to be ready

#     # ---------------------------------------------------------------------------------------
    
#     # ---------------------------- GO TO GOAL --------------------------------------
    
#     while my_robot.running and not rospy.is_shutdown():
#         user_input = input("Enter coordinates (x y) or q to quit: ")

#         if user_input.lower() == "q":
#             print("User exited.")
#             my_robot.running = False
#             break

#         try:
#             x, y = map(int, user_input.split())
#         except ValueError:
#             print("Please enter valid coordinates!")
#             continue

#         my_robot.goToGoal(x, y) 



#     # --------------------------------------------------------------------------------------------
    

#     # ---------------- VARIOUS TRAJECTORY FOLLOWING (USER-DEFINED) -------------------------------- 
#     # try:
#     #     I_trajectory = int(input("Select motion type: (1 -> Triangle, 2 -> Square, 3 -> Circle): "))
#     #     if I_trajectory not in [1, 2, 3]:
#     #         raise ValueError("Invalid selection")
#     # except ValueError as e:
#     #     print("Invalid selection:", e)
#     #     exit(1)

#     # # Get and validate linear velocity
#     # try:
#     #     I_linear_velocity = float(input("Enter linear velocity (0.1 - 1.0 m/s): "))
#     #     if not (0.1 <= I_linear_velocity <= 1.0):
#     #         raise ValueError("Must be between 0.1 - 1.0")
#     #     my_robot.linear_velocity = I_linear_velocity
#     # except ValueError as e:
#     #     print("Invalid velocity:", e)
#     #     exit(1)

#     # # Get parameters for polygon or circle
#     # if I_trajectory in [1, 2]:  # Polygon
#     #     try:
#     #         I_side_length = float(input("Enter side length (1 - 5 m): "))
#     #         if not (1 <= I_side_length <= 5):
#     #             raise ValueError("Must be between 1 - 5")
#     #         sides = 3 if I_trajectory == 1 else 4
#     #         print("Parameters received, robot is starting!\n")
#     #         my_robot.follow_polygon_trajectory(sides=sides, side_length=I_side_length)
#     #     except ValueError as e:
#     #         print("Invalid side length:", e)
#     #         exit(1)
#     # else:  # Circle
#     #     try:
#     #         I_radius = float(input("Enter circle radius (1 - 4 m): "))
#     #         if not (1 <= I_radius <= 4):
#     #             raise ValueError("Must be between 1 - 4")
#     #         print("Parameters received, robot is starting!\n")
#     #         my_robot.follow_circular_trajectory(radius=I_radius)
#     #     except ValueError as e:
#     #         print("Invalid radius:", e)
#     #         exit(1)

import rospy
from robot import Robot
from motion import Motion
from navigation import Navigation

if __name__ == "__main__":
    rospy.init_node("robot_controller", anonymous=True)

    my_robot = Robot()
    motion = Motion(my_robot)
    navigation = Navigation(my_robot)

    if my_robot.get_initial_position() is None:
        print("Cannot start, odometry not available!")
        exit(1)

    rospy.sleep(1)

    # ---------------------------- GO TO GOAL --------------------------------------
    while my_robot.running and not rospy.is_shutdown():
        user_input = input("Enter coordinates (x y) or q to quit: ")

        if user_input.lower() == "q":
            print("User exited.")
            my_robot.running = False
            break

        try:
            x, y = map(int, user_input.split())
        except ValueError:
            print("Please enter valid coordinates!")
            continue

        navigation.goToGoal(x, y)
    # --------------------------------------------------------------------------------

    # ---------------- VARIOUS TRAJECTORY FOLLOWING (USER-DEFINED) -------------------
    # Example usage (commented out):
    # try:
    #     I_trajectory = int(input("Select motion type: (1 -> Triangle, 2 -> Square, 3 -> Circle): "))
    #     if I_trajectory not in [1, 2, 3]:
    #         raise ValueError("Invalid selection")
    # except ValueError as e:
    #     print("Invalid selection:", e)
    #     exit(1)

    # try:
    #     I_linear_velocity = float(input("Enter linear velocity (0.1 - 1.0 m/s): "))
    #     if not (0.1 <= I_linear_velocity <= 1.0):
    #         raise ValueError("Must be between 0.1 - 1.0")
    #     my_robot.linear_velocity = I_linear_velocity
    # except ValueError as e:
    #     print("Invalid velocity:", e)
    #     exit(1)

    # if I_trajectory in [1, 2]:  # Polygon
    #     try:
    #         I_side_length = float(input("Enter side length (1 - 5 m): "))
    #         if not (1 <= I_side_length <= 5):
    #             raise ValueError("Must be between 1 - 5")
    #         sides = 3 if I_trajectory == 1 else 4
    #         print("Parameters received, robot is starting!\n")
    #         motion.follow_polygon_trajectory(sides=sides, side_length=I_side_length)
    #     except ValueError as e:
    #         print("Invalid side length:", e)
    #         exit(1)
    # else:  # Circle
    #     try:
    #         I_radius = float(input("Enter circle radius (1 - 4 m): "))
    #         if not (1 <= I_radius <= 4):
    #             raise ValueError("Must be between 1 - 4")
    #         print("Parameters received, robot is starting!\n")
    #         motion.follow_circular_trajectory(radius=I_radius)
    #     except ValueError as e:
    #         print("Invalid radius:", e)
    #         exit(1)
    # --------------------------------------------------------------------------------




