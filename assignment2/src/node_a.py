#! /usr/bin/env python3
import sys
import rospy
import actionlib

from Constants import OBJECTS_POSES, TAGS, MIDDLE_SAFE_POSITION, START_SAFE_POINT
from tiago_iaslab_simulation.srv import Objs, ObjsRequest, ObjsResponse
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from assignment1.msg import AssignmentAction, AssignmentGoal
from assignment2.msg import DetectAction, DetectGoal, ArmAction, ArmGoal


class NodeAClass(object):
    def __init__(self) -> None:
        rospy.loginfo("Main node got started.")

    def start(self, debug=True):
        tag_ids = self.get_sequences(debug=debug)
        self.go_to_position(START_SAFE_POINT)
        rospy.sleep(0.5)

        for tag_id in tag_ids:
            self.object_pose = None
            self.other_objects_pose = None
            self.other_objects_ids = None
            reached = False
            table_pose = OBJECTS_POSES[tag_id]["table"]
            safe_pose = OBJECTS_POSES[tag_id]["middle_point"]
            destination_pose = OBJECTS_POSES[tag_id]["cylinder"]
            rospy.loginfo(f"Tiago start moving to reach the table for {TAGS[tag_id]}.")
            self.go_to_position(safe_pose)
            rospy.sleep(0.5)
            reached = self.go_to_position(table_pose)
            if reached:
                rospy.loginfo("Tiago reached the goal.")
            rospy.sleep(0.5)
            self.detect_objects(tag_id)

            if self.object_pose:
                self.pick_and_place(tag_id, True)

                rospy.loginfo("Tiago start moving to reach the related cylinder.")
                self.go_to_position(safe_pose)
                self.go_to_position(MIDDLE_SAFE_POSITION)
                reached = self.go_to_position(destination_pose)
                if reached:
                    rospy.loginfo("Tiago reached the related cylinder.")
                rospy.sleep(3.0)

                # need to change
                self.pick_and_place(tag_id, False)

                self.go_to_position(MIDDLE_SAFE_POSITION)
            else:
                rospy.logerr(
                    "Tiago didn't find the object so it can't continue the process"
                )
                break

    def feedback_cb(self, msg):
        rospy.loginfo(msg.feedback)

    def get_sequences(self, debug=True):
        if debug:
            rospy.loginfo("You are runing in *DEBUG MODE*. It goes to id 1.")
            return (1,)

        rospy.wait_for_service("/human_objects_srv")
        try:
            human_objects_service = rospy.ServiceProxy("/human_objects_srv", Objs)
            response = human_objects_service(True, True)
            rospy.loginfo(
                f"secuence of objects received from human node: {response.ids}"
            )
            return response.ids
        except rospy.ServiceException as e:
            rospy.logerr(f"Human node service call failed: {e}")

    def go_to_position(self, pose):
        # if you want to use the action server of assignment1 uncomment the following block and run the related node
        # but its not recommended because it do unneccesery procces for detecting obstacles:)
        # client = actionlib.SimpleActionClient("assignment_server", AssignmentAction)

        # goal_position = MoveBaseGoal()
        # goal_position.target_pose.header.frame_id = "map"
        # goal_position.target_pose.header.stamp = rospy.Time.now()
        # goal_position.target_pose.pose = pose
        # client.wait_for_server()
        # client.send_goal(goal_position)

        # wait = client.wait_for_result()
        # result = client.get_state()
        # if not wait:
        #     rospy.signal_shutdown("Move base action server not available!")
        #     return False

        # elif result != actionlib.GoalStatus.SUCCEEDED:
        #     # Result of executing the action
        #     rospy.loginfo(f"Tiago cant reach to goal with status code {result}.")
        #     return False
        # else:
        #     # Result of executing the action
        #     # rospy.loginfo("Tiago reached the goal.")
        #     return True

        # Creates the SimpleActionClient

        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        goal_position = MoveBaseGoal()
        goal_position.target_pose.header.frame_id = "map"
        goal_position.target_pose.header.stamp = rospy.Time.now()
        goal_position.target_pose.pose = pose
        client.wait_for_server()
        # Sends the goal to the action server.
        client.send_goal(goal_position)

        wait = client.wait_for_result()

        result = client.get_state()
        if not wait:
            rospy.signal_shutdown("Move base action server not available!")
            return False

        elif result != actionlib.GoalStatus.SUCCEEDED:
            # Result of executing the action
            rospy.loginfo(f"Tiago cant reach to goal with status code {result}.")
            return False
        else:
            # Result of executing the action
            # rospy.loginfo("Tiago reached the goal.")
            return True

    def detect_objects(self, tag_id):
        rospy.loginfo("Waiting for Detect Action Server.")
        client = actionlib.SimpleActionClient("detect_server", DetectAction)
        client.wait_for_server()
        goal = DetectGoal(tag_id)
        client.send_goal(goal, feedback_cb=self.feedback_cb)
        rospy.loginfo(f"ID: {tag_id} sent to Detect Action Server.")
        wait = client.wait_for_result()
        if not wait:
            rospy.loginfo("Detect Action server is not available!")
            rospy.signal_shutdown("Detect Action Server is not available!")
        else:
            rospy.loginfo("Result received from Detect Action server.")
            result = client.get_result()
            self.object_pose = result.object_pose
            self.other_objects_pose = result.other_objects_pose
            self.other_objects_ids = result.other_objects_ids

    def pick_and_place(self, object_id, pick):
        rospy.loginfo("Waiting for Detect Action Server.")
        client = actionlib.SimpleActionClient("arm_server", ArmAction)
        client.wait_for_server()

        goal = ArmGoal()
        goal.pick = pick
        goal.object_pose = self.object_pose
        goal.other_objects_pose = self.other_objects_pose
        goal.other_objects_ids = self.other_objects_ids
        goal.object_id = object_id

        client.send_goal(goal, feedback_cb=self.feedback_cb)
        rospy.loginfo("Goal sent to Arm Action Server.")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Arm Action server is not available!")
            rospy.signal_shutdown("Arm Action Server is not available!")
        else:
            rospy.loginfo("Result received from Arm Action server.")
            result = client.get_result()
            return result


if __name__ == "__main__":
    try:
        rospy.init_node("main_node")
        debug_arg = rospy.get_param("~DEBUG", True)
        node_a = NodeAClass()
        node_a.start(debug=debug_arg)
        rate = rospy.Rate(100)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
