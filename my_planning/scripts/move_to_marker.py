#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import MarkerArray, Marker
import tf.transformations as tf


def marker_callback(msg):
    print("Received marker message")  # Add this line

    if len(msg.markers) > 0:
        # Assuming you want the pose of the first marker in the array
        target_marker: Marker = msg.markers[0]

        # Create a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header = target_marker.header
        pose_stamped.pose = target_marker.pose

        # Adjust the orientation of the pose to face towards the marker
        flipped_quaternion = tf.quaternion_multiply(
            [1, 0, 0, 0],
            [
                target_marker.pose.orientation.x,
                target_marker.pose.orientation.y,
                target_marker.pose.orientation.z,
                target_marker.pose.orientation.w,
            ],
        )
        pose_stamped.pose.orientation.x = flipped_quaternion[0]
        pose_stamped.pose.orientation.y = flipped_quaternion[1]
        pose_stamped.pose.orientation.z = flipped_quaternion[2]
        pose_stamped.pose.orientation.w = flipped_quaternion[3]

        # Publish the PoseStamped message to the target_pose topic
        target_pose_pub.publish(pose_stamped)

        # Unsubscribe from the aruco_marker topic
        aruco_marker_sub.unregister()


if __name__ == "__main__":
    rospy.init_node("marker_pose_publisher")
    # Create a publisher for the target_pose topic
    target_pose_pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=10)

    rospy.sleep(1)  # Add a delay of 1 second

    # Subscribe to the aruco_marker topic
    aruco_marker_sub = rospy.Subscriber("/aruco_markers", MarkerArray, marker_callback)

    rospy.spin()
