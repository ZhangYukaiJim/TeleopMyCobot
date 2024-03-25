#!/usr/bin/env python

import rospy, roslib, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MoveItPlanner:
    def __init__(self):
        # API to initialize move_group,初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node,初始化ROS节点
        rospy.init_node("motion_planner")

        # Initialize the scene object to monitor changes in the external environment
        # 初始化场景对象，用来监听外部环境的变化
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)

        # Initialize self.arm group in the robotic arm that needs to be controlled by move group
        # 初始化需要使用move group控制的机械臂中的self.arm group
        self.arm = moveit_commander.MoveGroupCommander("arm_group")

        # Get the name of the terminal link,获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference coordinate system used for the target position
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = "joint1"
        self.arm.set_pose_reference_frame(self.reference_frame)

        # Allow replanning when motion planning fails,当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)

        # Set the allowable error of position (unit: meter) and attitude (unit: radian)
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)

        # Set the current state of the robot arm as the initial state of motion
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        # Setup the chasis for obstacle avoidance
        self.scene.remove_world_object("suit")
        quat = quaternion_from_euler(1.57, 0, -1.57)
        suit_post = PoseStamped()
        suit_post.header.frame_id = self.reference_frame
        suit_post.pose.position.x = 0.0
        suit_post.pose.position.y = 0.0
        suit_post.pose.position.z = -0.00
        suit_post.pose.orientation.x = quat[0]
        suit_post.pose.orientation.y = quat[1]
        suit_post.pose.orientation.z = quat[2]
        suit_post.pose.orientation.w = quat[3]
        suit_path = (
            roslib.packages.get_pkg_dir("mycobot_description")
            + "/urdf/teleop/chasis.obj"
        )
        self.scene.add_mesh("suit", suit_post, suit_path)

        # Listen to the topic for target_pose
        rospy.Subscriber("target_pose", PoseStamped, self.callback)
        rospy.spin()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(data.data, self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj[1])
        rospy.sleep(1)


if __name__ == "__main__":
    planner = MoveItPlanner()
