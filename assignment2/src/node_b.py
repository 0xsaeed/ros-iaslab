#! /usr/bin/env python3
import rospy
import actionlib
import sys
import cv2
import tf2_ros

from Constants import TAGS
from assignment2.msg import DetectAction, DetectResult, DetectFeedback
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_geometry_msgs import do_transform_pose
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError


class DetectionActionClass(object):
    def __init__(self, name) -> None:
        self.detected_objects = None
        self.april_tag_result = None
        self.head = rospy.Publisher(
            "/head_controller/command", JointTrajectory, queue_size=1
        )
        rospy.sleep(1)

        self.torso_cmd = rospy.Publisher(
            "/torso_controller/command", JointTrajectory, queue_size=1
        )
        rospy.sleep(1)

        self.camera = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.camera_cb)
        rospy.sleep(1)

        self.april_tag = rospy.Subscriber(
            "/tag_detections", AprilTagDetectionArray, self.april_tag_cb
        )
        rospy.sleep(1)

        self._as_feedback = DetectFeedback()
        self._as_result = DetectResult()
        self._as = actionlib.SimpleActionServer(
            name,
            DetectAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.sleep(1)

    def execute_cb(self, goal):
        if self.camera:
            rospy.loginfo("Subscribed to Camera topic successfully")
        rospy.sleep(0.5)
        if self.april_tag:
            rospy.loginfo("Subscribed to Tag Detection topic successfully")
        rospy.sleep(0.5)

        self.lift_torso()
        rospy.sleep(0.5)

        self.feedback_cb("Start moving head.")
        self.move_head()
        rospy.sleep(3)
        self.feedback_cb("Detection got started.")
        # because of race condition
        self.detected_objects = self.april_tag_result
        while not self.detected_objects:
            self.feedback_cb("Wating for tag april tag node to detect goal object.")
            rospy.sleep(5)
            self.detected_objects = self.april_tag_result

        if goal.tag_id in self.detected_objects.keys():
            self.feedback_cb(f"{TAGS[goal.tag_id]} found.")
            for object_id in self.detected_objects.keys():
                transformed_pose = self.transform(self.detected_objects[object_id].pose)
                if object_id == goal.tag_id:
                    self._as_result.object_pose = transformed_pose
                else:
                    self._as_result.other_objects_pose.append(transformed_pose)
                    self._as_result.other_objects_ids.append(object_id)
            self._as.set_succeeded(self._as_result)
        else:
            # just for test
            for object_id in self.detected_objects.keys():
                self.feedback_cb("Goal object not found but found some other objects")
                transformed_pose = self.transform(self.detected_objects[object_id].pose)
                self._as_result.other_objects_pose.append(transformed_pose)
                self._as_result.other_objects_ids.append(object_id)
            print(self._as_result.other_objects_pose)
            self.feedback_cb(f"Couldn't find the {TAGS[goal.tag_id]} object")
            self._as.set_aborted(self._as_result)

    def transform(self, pose_stamped_with_covariance):
        pose_stamped = PoseStamped()
        pose_stamped.header = pose_stamped_with_covariance.header
        pose_stamped.pose = pose_stamped_with_covariance.pose.pose
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            tf_buffer.can_transform(
                "base_footprint",
                pose_stamped.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            transform = tf_buffer.lookup_transform(
                "base_footprint", pose_stamped.header.frame_id, rospy.Time(0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("Failed to lookup transform: {}".format(e))
            return None

        # Transform the pose
        pose_tag_in_base = do_transform_pose(pose_stamped, transform)

        return pose_tag_in_base

    def move_head(self, pose=[0, -0.9]):
        head_trajectory = JointTrajectory()
        head_trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = pose
        point.time_from_start = rospy.Duration(2.0)
        head_trajectory.points.append(point)
        # head_trajectory.header.stamp = rospy.Time.now()
        self.head.publish(head_trajectory)
        self.feedback_cb("Head moved to desire point.")

    def feedback_cb(self, msg):
        self._as_feedback.feedback = "[Detect Action Server] " + msg
        self._as.publish_feedback(self._as_feedback)

    def camera_cb(self, msg):
        try:
            cv2_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Camera Error :{e}")
            print(e)
            return False
        else:
            cv2.imshow("Camera", cv2_img)
            cv2.waitKey(1)
            return True

    def lift_torso(self):
        # Move torso to its maximum height
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ["torso_lift_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.30]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)

    def april_tag_cb(self, msg):
        self.april_tag_result = {i.id[0]: i for i in msg.detections}


if __name__ == "__main__":
    try:
        rospy.init_node("detect_server")
        rate = rospy.Rate(100)
        server = DetectionActionClass(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
