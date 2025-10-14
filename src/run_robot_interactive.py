#!/usr/bin/env python3
"""
run_robot.py
Clean and simple main program - only user interface and coordination
"""

import rospy
from robot import Robot
from navigation import Navigation


# def main():
#     rospy.init_node('run_robot_clean', anonymous=True)
    
#     # Start robot and navigation system
#     robot = Robot()
#     navigation = Navigation(robot)
    
#     rospy.sleep(1.0)  # Wait for initialization
#     rate = rospy.Rate(10)  # 10 Hz
    
#     print("Robot ready! Enter coordinates (x y) or 'q' to quit.")
    
#     try:
#         while not rospy.is_shutdown():
            
#             # If no goal or goal reached, request new goal
#             if not navigation.has_goal():
#                 user_input = input("Target coordinates (x y) or q: ").strip()
                
#                 if user_input.lower() == 'q':
#                     break
                
#                 try:
#                     x, y = map(float, user_input.split())
#                     navigation.set_goal(x, y)
#                 except Exception:
#                     print("Invalid format! Example: 1.5 2.0")
#                     continue
            
#             # Navigation update and movement
#             twist = navigation.update()
#             robot.cmd_vel_pub.publish(twist)
            
#             rate.sleep()# --- Kodun Geri Kalanı Neredeyse Aynı ---
            
#     except rospy.ROSInterruptException:
#         pass
#     except KeyboardInterrupt:
#         print("\nExiting...")
#     finally:
#         # Stop robot
#         stop_twist = navigation.stop()
#         robot.cmd_vel_pub.publish(stop_twist)
#         print("Robot stopped.")


# if __name__ == "__main__":
#     main()


"""
run_robot_interactive.py
An interactive script for manual testing.
It allows the user to select an algorithm and set goals dynamically.
"""

import rospy
from robot import Robot
from navigation import Navigation

def main():
    # Initialize the ROS node with a unique name
    rospy.init_node('run_robot_interactive', anonymous=True)
    
    
    print("------------------------------------")
    print("--- Algorithm Selection ---")
    algorithm_choice = input("Select an algorithm (bug0, bug1, bug2) [default is bug0]: ").strip().lower()
    
    # Check if the user's input is valid. If not, use the safe default.
    if algorithm_choice not in ["bug0", "bug1", "bug2"]:
        if algorithm_choice: # If the user typed something invalid
            print(f"Invalid selection '{algorithm_choice}'. Defaulting to 'bug0'.")
        else: # If the user just pressed Enter
            print("No selection made. Defaulting to 'bug0'.")
        selected_algorithm = "bug0"
    else:
        selected_algorithm = algorithm_choice
    
    print(f"Algorithm '{selected_algorithm.upper()}' selected.")
    print("------------------------------------")
    
    
    
    # Create the Robot and Navigation instances.
    robot = Robot()
    
    # CRITICAL CHANGE: Pass the selected algorithm to the Navigation brain!
    navigation = Navigation(robot, algorithm=selected_algorithm)
    
    rospy.sleep(1.0)  # Wait for everything to initialize
    rate = rospy.Rate(10)  # 10 Hz
    
    print("Robot ready! Enter coordinates (x y) or 'q' to quit.")
    
    try:
        while not rospy.is_shutdown():
            
            # If there is no active goal, ask the user for a new one.
            if not navigation.has_goal():
                user_input = input("Target coordinates (x y) or q: ").strip()
                
                if user_input.lower() == 'q':
                    break
                
                try:
                    x, y = map(float, user_input.split())
                    navigation.set_goal(x, y)
                except Exception:
                    print("Invalid format! Example: 1.5 2.0")
                    continue
            
            # Main navigation update loop
            twist = navigation.update()
            robot.cmd_vel_pub.publish(twist)
            
            rate.sleep()
            
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        # Handle Ctrl+C gracefully
        print("\nExiting...")
    finally:
        # Always stop the robot when the program ends.
        stop_twist = navigation.stop()
        robot.cmd_vel_pub.publish(stop_twist)
        print("Robot stopped.")


if __name__ == "__main__":
    main()


# ------------------------ VARIOUS TRAJECTORY EXAMPLES -------------------------------------
#     # Example usage:
#     #my_robot.follow_polygon_trajectory(sides=4, side_length=3)      # Square
#     #my_robot.follow_polygon_trajectory(sides=3, side_length=2)      # Triangle
#     #my_robot.follow_circular_trajectory(radius=1.5)                # Circle
# --------------------------------------------------------------------------------------
# ----------------------- COMMON NODE START ----------------------------------------
#     rospy.init_node("robot_controller", anonymous=True)
#     my_robot = Robot()
#     # Check odometry
#     if my_robot.get_initial_position() is None:
#         print("Cannot start, odometry not available!")
#         exit(1)
#     rospy.sleep(1)  # Wait for subscribers to be ready
# ---------------------------------------------------------------------------------------

# ---------------------------- GO TO GOAL --------------------------------------

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
# --------------------------------------------------------------------------------------------

# ---------------- VARIOUS TRAJECTORY FOLLOWING (USER-DEFINED) -------------------------------- 
# try:
#     I_trajectory = int(input("Select motion type: (1 -> Triangle, 2 -> Square, 3 -> Circle): "))
#     if I_trajectory not in [1, 2, 3]:
#         raise ValueError("Invalid selection")
# except ValueError as e:
#     print("Invalid selection:", e)
#     exit(1)
# # Get and validate linear velocity
# try:
#     I_linear_velocity = float(input("Enter linear velocity (0.1 - 1.0 m/s): "))
#     if not (0.1 <= I_linear_velocity <= 1.0):
#         raise ValueError("Must be between 0.1 - 1.0")
#     my_robot.linear_velocity = I_linear_velocity
# except ValueError as e:
#     print("Invalid velocity:", e)
#     exit(1)
# # Get parameters for polygon or circle
# if I_trajectory in [1, 2]:  # Polygon
#     try:
#         I_side_length = float(input("Enter side length (1 - 5 m): "))
#         if not (1 <= I_side_length <= 5):
#             raise ValueError("Must be between 1 - 5")
#         sides = 3 if I_trajectory == 1 else 4
#         print("Parameters received, robot is starting!\n")
#         my_robot.follow_polygon_trajectory(sides=sides, side_length=I_side_length)
#     except ValueError as e:
#         print("Invalid side length:", e)
#         exit(1)
# else:  # Circle
#     try:
#         I_radius = float(input("Enter circle radius (1 - 4 m): "))
#         if not (1 <= I_radius <= 4):
#             raise ValueError("Must be between 1 - 4")
#         print("Parameters received, robot is starting!\n")
#         my_robot.follow_circular_trajectory(radius=I_radius)
#     except ValueError as e:
#         print("Invalid radius:", e)
#         exit(1)
# ---------------------------------------------------------------------------------------------