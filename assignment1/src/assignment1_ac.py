#! /usr/bin/env python3
import sys
import rospy
import actionlib

from geometry_msgs.msg import Point, Pose, Quaternion
from assignment1.msg import AssignmentAction, AssignmentGoal


def get_postion() -> Pose:
    x_position = float(input("Enter x Position of Goal: "))
    y_position = float(input("Enter y Position of Goal: "))
    z_orientation = float(input("Enter z Orientation of Goal: "))
    # while z_orientation
    position = Pose(Point(x=x_position, y=y_position), Quaternion(z=z_orientation, w=1))

    return position


def feedback_callback(msg):
    rospy.loginfo(msg)


def assignment_client(pose_b: Pose):
    # Creates the SimpleActionClient
    rospy.loginfo(f"The client node got this position as a goal:\n{pose_b}")
    client = actionlib.SimpleActionClient("assignment_server", AssignmentAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()
    goal = AssignmentGoal(pose_b)
    # Sends the goal to the assignment action server.
    client.send_goal(goal, feedback_cb=feedback_callback)
    rospy.loginfo("Goal sent to the assignment action server.")

    wait = client.wait_for_result()
    # Waits for the server to finish performing the action.
    if not wait:
        rospy.logerr("Action server is not available!")
        rospy.signal_shutdown("Action server is not available!")
    else:
        result = client.get_result()
        if result:
            # Access the obstacles from the result and log them
            obstacles = result.obstacles
            rospy.loginfo(
                "Detected obstacles position with respect to tiago current position:"
            )
            for i, obstacle in enumerate(obstacles):
                # Format x and y coordinates to display only two decimal places
                x_formatted = "{:.2f}".format(obstacle.x)
                y_formatted = "{:.2f}".format(obstacle.y)
                rospy.loginfo(f"Obstacle {i + 1}: (x={x_formatted}, y={y_formatted})")
        else:
            rospy.loginfo("Action failed with no result")


if __name__ == "__main__":
    try:
        pose_b = get_postion()
        rospy.init_node("assignment_client")
        rate = rospy.Rate(10)
        result = assignment_client(pose_b)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
