#! /usr/bin/env python3

import numpy as np
import sys
import rospy
import actionlib

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from assignment1.msg import AssignmentAction, AssignmentFeedback, AssignmentResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveTiagoActionClass(object):
    _feedback = AssignmentFeedback()
    _result = AssignmentResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            AssignmentAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.ranges = []
        self.angles = []
        self._as.start()
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        # Store the ranges and angles from the Scan message
        self.ranges = msg.ranges
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

    def detection(self) -> list:
        # Obstacle detecting part

        obstacles = list()

        # Calculate the gradient of the Scan ranges
        range_gradient = np.gradient(self.ranges)

        # Set a threshold for detecting sudden changes in gradient
        gradient_threshold = 0.3

        # Find indices where the gradient exceeds the threshold
        change_indices = np.where(np.abs(range_gradient) > gradient_threshold)[0]

        # Iterate through detected changes to find side points for each obstacle
        for i in range(len(change_indices) - 1):
            first_point_index = change_indices[i]
            last_point_index = change_indices[i + 1]
            # Check the difference between intervals(first and last point)
            point_difference = last_point_index - first_point_index

            # It considers obstacles with 5 to 60 points width
            if (range_gradient[first_point_index] < 0) and range_gradient[
                last_point_index
            ] > 0:
                if 5 <= point_difference <= 60:
                    # Calculate the x and y coordinates of the pair points
                    x1 = self.ranges[first_point_index] * np.cos(
                        self.angles[first_point_index]
                    )
                    y1 = self.ranges[first_point_index] * np.sin(
                        self.angles[first_point_index]
                    )
                    x2 = self.ranges[last_point_index] * np.cos(
                        self.angles[last_point_index]
                    )
                    y2 = self.ranges[last_point_index] * np.sin(
                        self.angles[last_point_index]
                    )

                    # Calculate the average of x and y coordinates as center of the obstacle
                    average_x = (x1 + x2) / 2
                    average_y = (y1 + y2) / 2

                    obstacle = Point()
                    obstacle.x = average_x
                    obstacle.y = average_y
                    obstacles.append(obstacle)

        return obstacles

    def execute_cb(self, goal):
        # Maybe it sounds wierd but in this action server we create a client
        # to send a goal to move_base server:))

        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Creates a goal to send to the action server.

        goal_position = MoveBaseGoal()
        goal_position.target_pose.header.frame_id = "map"
        goal_position.target_pose.header.stamp = rospy.Time.now()
        goal_position.target_pose.pose.position.x = goal.pose_b.position.x
        goal_position.target_pose.pose.position.y = goal.pose_b.position.y
        goal_position.target_pose.pose.orientation.z = goal.pose_b.orientation.z
        goal_position.target_pose.pose.orientation.w = goal.pose_b.orientation.w
        self._feedback.state = "Waiting for move_base action server to come up..."
        self._as.publish_feedback(self._feedback)

        # Waits until the action server has started up and started
        # listening for goals.

        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal_position)
        self._feedback.state = "Goal has been sent to move_base action server."
        self._as.publish_feedback(self._feedback)
        self._feedback.state = "Tiago is moving to goal."
        self._as.publish_feedback(self._feedback)

        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()

        result = client.get_state()
        if not wait:
            self._feedback.state = "move_base Action server not available!"
            self._as.publish_feedback(self._feedback)
            rospy.signal_shutdown("Action server not available!")
            return

        elif result != actionlib.GoalStatus.SUCCEEDED:
            # Result of executing the action
            self._feedback.state = (
                f"Tiago cant reach to goal with status code {result}."
            )
            self._as.publish_feedback(self._feedback)
            return

        else:
            # Result of executing the action
            self._feedback.state = "Tiago reached to goal."
            self._as.publish_feedback(self._feedback)

            rospy.sleep(1.0)
            self._feedback.state = "Detection started"
            self._as.publish_feedback(self._feedback)

            rospy.sleep(1.0)
            obstacles = self.detection()
            self._result.obstacles = obstacles
            self._feedback.state = "Detection finished"
            self._as.publish_feedback(self._feedback)

            rospy.sleep(1.0)
            self._feedback.state = f"It found {len(obstacles)} obstacle/s"
            self._as.publish_feedback(self._feedback)

            rospy.sleep(1.0)
            self._feedback.state = "Proccess is finished."
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)


if __name__ == "__main__":
    try:
        rospy.init_node("assignment_server")
        rate = rospy.Rate(10)
        server = MoveTiagoActionClass(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
