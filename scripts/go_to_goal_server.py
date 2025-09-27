#!/usr/bin/env python3

import threading
import rospy
import actionlib
import math

# Package imports (paket adiyla)
from turtlebot3_trajectory_pkg.msg import GoToGoalAction, GoToGoalFeedback, GoToGoalResult
from turtlebot3_trajectory_pkg.navigation import Navigation
from turtlebot3_trajectory_pkg.robot import Robot


class GoToGoalServer:
    def __init__(self):
        rospy.init_node('go_to_goal_server')

        self.robot = Robot()
        self.navigation = Navigation(self.robot)

        self.server = actionlib.SimpleActionServer(
            'go_to_goal',
            GoToGoalAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()

        self._feedback_rate_hz = 5.0
        rospy.loginfo("[go_to_goal_server] Server ready.")

    def _feedback_publisher(self, goal_x, goal_y, stop_event: threading.Event):
        feedback = GoToGoalFeedback()
        rate = rospy.Rate(self._feedback_rate_hz)

        while not stop_event.is_set() and not rospy.is_shutdown():
            if getattr(self.robot, "position", None) is None:
                feedback.distance_remaining = float('inf')
            else:
                dx = goal_x - self.robot.position.x
                dy = goal_y - self.robot.position.y
                feedback.distance_remaining = math.sqrt(dx**2 + dy**2)
            try:
                self.server.publish_feedback(feedback)
            except Exception:
                pass
            rate.sleep()

    def execute_cb(self, goal):
        rospy.loginfo(f"[Server] Received goal: x={goal.x}, y={goal.y}")

        stop_event = threading.Event()
        feedback_thread = threading.Thread(
            target=self._feedback_publisher,
            args=(goal.x, goal.y, stop_event),
            daemon=True
        )
        feedback_thread.start()

        success = False
        try:
            self.navigation.goToGoal(goal.x, goal.y)
            success = True
        except Exception as e:
            rospy.logerr(f"[Server] Exception: {e}")
            try:
                self.robot.stop()
            except Exception:
                pass
        finally:
            stop_event.set()
            feedback_thread.join(timeout=1.0)

        result = GoToGoalResult()
        result.success = success
        if success:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

if __name__ == "__main__":
    server = GoToGoalServer()
    rospy.spin()
