#!/usr/bin/env python3
"""
robot.py
Robot sensor data and actuator management module
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class Robot:
    def __init__(self):
        # State variables
        self.position = None
        self.yaw = None
        self.odom = None
        
        self.twist = Twist()
        self.lidar_ranges = []
        
        # Publishers / Subscribers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.odom = msg
        self.position = msg.pose.pose.position
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.yaw = euler_from_quaternion(q)[2]
    
    def lidar_callback(self, scan):
        """Process lidar data"""
        self.lidar_ranges = list(scan.ranges)
    
    def get_front_distance(self, win=30):
        """Return min distance in ±win deg around the front"""
        if not self.lidar_ranges:
            return float("inf")
        a = self.lidar_ranges[0:win]
        b = self.lidar_ranges[-win:]
        return min(a + b)
    
    def get_left_distance(self, start=60, end=120):
        """Return minimum distance in left region"""
        if not self.lidar_ranges:
            return float("inf")
        return min(self.lidar_ranges[start:end])
    
    def stop(self):
        """Stop the robot"""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(0.1)
