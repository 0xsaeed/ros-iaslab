#! /usr/bin/env python3
import sys
import rospy
import actionlib
from moveit_commander import (
    RobotCommander,
    MoveGroupCommander,
    PlanningSceneInterface,
    roscpp_initialize,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import CollisionObject, PlanningScene
from geometry_msgs.msg import Pose, PoseStamped
from assignment2.msg import ArmAction, ArmActionFeedback, ArmActionResult
from Constants import TAGS


class ArmActionClass(object):
    def __init__(self, name) -> None:
        self.torso_cmd = rospy.Publisher(
            "/torso_controller/command", JointTrajectory, queue_size=1
        )
        self.scene = PlanningSceneInterface()

        self._as_feedback = ArmActionFeedback()
        self._as_result = ArmActionResult()
        self._as = actionlib.SimpleActionServer(
            name,
            ArmAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        # Initialize MoveIt
        self.robot = RobotCommander()
        self.arm = MoveGroupCommander("arm")
        self._as.start()

    def execute_cb(self, goal):
        if goal.pick:
            self.pick(goal.object_pose, goal.other_objects_pose, goal.other_objects_ids)
        else:
            self.place()

    def feedback_cb(self, msg):
        rospy.loginfo(msg)
        self._as_feedback.feedback = "[Arm Action Server] " + msg
        self._as.publish_feedback(self._as_feedback)

    def pick(self, object_pose, other_objects_pose, other_objects_ids):
        self.update_scene(other_objects_pose, other_objects_ids)
        print(object_pose)
        self.arm.set_pose_target(object_pose, end_effector_link="gripper")
        plan_success, plan, planning_time, error_code = self.arm.plan()
        rospy.sleep(5)
        if plan_success:
            self.arm.execute(plan)
            self.feedback_cb("Picked object successfully")
            self.tuck()
            self.scene.clear()
        else:
            self.scene.clear()
            self.feedback_cb("Planning for picking object was unsuccessful")
            self._as.set_aborted()

    def place(self):
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = 0.5
        pose_stamped.pose.position.y = 0.5
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        pose_stamped.header.frame_id = "base_footprint"
        pose_stamped.header.stamp = rospy.Time.now()
        self.scene.clear()
        self.arm.set_pose_target(pose_stamped, end_effector_link="gripper")
        plan_success, plan, planning_time, error_code = self.arm.plan()
        if plan_success:
            self.arm.execute(plan)
            self.feedback_cb("Placed object successfully")
            self.tuck()
        else:
            self.feedback_cb("Planning for placing object was unsuccessful")
            self._as.set_aborted()

    def tuck(self):
        rospy.loginfo("Closing the arm")
        tuck_pose = PoseStamped()
        tuck_pose.pose.position.x = 0.05
        tuck_pose.pose.position.y = 0.05
        tuck_pose.pose.position.z = 0.05
        tuck_pose.header.frame_id = "base_footprint"
        tuck_pose.header.stamp = rospy.Time.now()
        self.arm.set_pose_target(tuck_pose, end_effector_link="gripper")
        plan_success, plan, planning_time, error_code = self.arm.plan()
        self.arm.execute(plan)
        self.feedback_cb("Arm Tucked successfully")

    def update_scene(self, other_objects_pose, other_objects_ids):
        # Add a table as a collision object
        table_size = [1.0, 1.5, 0.8]
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_footprint"
        table_pose.pose.position.x = 0.9
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.38
        self.scene.add_box("table", table_pose, table_size)
        for i, obj_id in enumerate(other_objects_ids):
            obj_pose = other_objects_pose[i]
            height = 0.1
            radius = 0.05
            obj_pose.pose.position.z -= height / 2
            print(obj_pose)
            self.scene.add_cylinder(TAGS[obj_id], obj_pose, height, radius)


if __name__ == "__main__":
    try:
        rospy.init_node("arm_server")
        rate = rospy.Rate(10)
        roscpp_initialize(sys.argv)
        server = ArmActionClass(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)