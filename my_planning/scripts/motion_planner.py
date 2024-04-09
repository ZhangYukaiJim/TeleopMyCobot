#!/usr/bin/env python

import rospy, roslib, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener


class MoveItPlanner:
    def __init__(self):
        # API to initialize move_group
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node("motion_planner")

        # Initialize the scene object to monitor changes in the external environment
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)

        # Initialize self.arm group in the robotic arm that needs to be controlled by move group
        self.arm = moveit_commander.MoveGroupCommander("arm_group")

        # Get the name of the terminal link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference coordinate system used for the target position
        self.reference_frame = "joint1"
        self.arm.set_pose_reference_frame(self.reference_frame)

        # Allow replanning when motion planning fails
        self.arm.allow_replanning(True)

        # Set the allowable error of position (unit: meter) and attitude (unit: radian)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)

        # Set the current state of the robot arm as the initial state of motion
        self.arm.set_start_state_to_current_state()

        # Initialize the TransformListener to listen to the transformation between frames
        self.tf_listener = TransformListener()

        # Listen to the topic for target_pose
        rospy.Subscriber("target_pose", PoseStamped, self.callback)
        rospy.spin()

    def transform_pose_to_reference(self, pose_stamped):
        try:
            # Transform the pose to the reference frame
            self.tf_listener.waitForTransform(
                pose_stamped.header.frame_id,
                self.reference_frame,
                rospy.Time(),
                rospy.Duration(1.0),
            )
            transformed_pose = self.tf_listener.transformPose(
                self.reference_frame, pose_stamped
            )
            print("Transformed Pose: ")
            print(transformed_pose)
            return transformed_pose
        except Exception as e:
            rospy.logerr("Failed to transform pose: %s", str(e))
            return None

    def callback(self, pose_stamped):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose_stamped)

        # Transform the received pose to the reference frame
        transformed_pose = self.transform_pose_to_reference(pose_stamped)
        if transformed_pose is None:
            rospy.logerr("Failed to transform pose. Aborting motion planning.")
            return

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(transformed_pose.pose, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj[1])
        rospy.sleep(1)


if __name__ == "__main__":
    planner = MoveItPlanner()
