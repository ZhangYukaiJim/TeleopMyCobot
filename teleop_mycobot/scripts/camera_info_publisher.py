#!/usr/bin/env python

import rospy
import yaml
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher:
    def __init__(self):
        rospy.init_node("camera_info_publisher", anonymous=True)
        self.camera_info_pub = rospy.Publisher(
            "/camera/camera_info", CameraInfo, queue_size=10
        )
        self.camera_info_loaded = False

        camera_info_file = rospy.get_param("~camera_info_file", None)
        if camera_info_file is not None:
            self.load_camera_info(camera_info_file)
            self.camera_info_loaded = True
        else:
            rospy.logerr(
                "No camera_info_file parameter specified. CameraInfo publisher will not start."
            )

    def load_camera_info(self, camera_info_file):
        with open(camera_info_file, "r") as f:
            self.camera_info = yaml.safe_load(f)

    def publish_camera_info(self):
        if not self.camera_info_loaded:
            rospy.logwarn(
                "Camera info not loaded. Skipping publishing CameraInfo message."
            )
            return

        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_msg.header.frame_id = "camera_frame"  # Set the frame ID here
        camera_info_msg.width = self.camera_info["image_width"]
        camera_info_msg.height = self.camera_info["image_height"]
        camera_info_msg.K = self.camera_info["camera_matrix"]["data"]
        camera_info_msg.D = self.camera_info["distortion_coefficients"]["data"]
        camera_info_msg.R = self.camera_info["rectification_matrix"]["data"]
        camera_info_msg.P = self.camera_info["projection_matrix"]["data"]
        self.camera_info_pub.publish(camera_info_msg)


if __name__ == "__main__":
    try:
        camera_info_publisher = CameraInfoPublisher()
        rate = rospy.Rate(10)  # Define the publishing rate
        while not rospy.is_shutdown():
            camera_info_publisher.publish_camera_info()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
