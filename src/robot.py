#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Robot:
    def __init__(self):
        # Base variables
        self.twist = Twist()
        self.position = None
        self.orientation = None
        self.yaw = None
        self.odom = None
        self.running = True

        # Velocities
        self.linear_velocity = 0.2
        self.angular_velocity = 0.5

        # ROS Publisher and Subscriber
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

    # Getting Odometry Data
    def get_initial_position(self, timeout=5):
        import time
        start_time = time.time()
        while self.odom is None:
            if time.time() - start_time > timeout:
                print("Odometry data not received!")
                return None
            rospy.sleep(0.1)
        return self.odom

    # Odometry callback
    def odom_callback(self, msg):
        self.odom = msg
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        q = (
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.orientation.w
        )
        self.yaw = euler_from_quaternion(q)[2]

    # Stop the robot
    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(0.1)
