#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf_conversions
import geometry_msgs.msg


class ImageConverter:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.bridge = CvBridge()
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        self.parameters = cv.aruco.DetectorParameters_create()
        self.marker_publisher = rospy.Publisher(
            "/aruco_markers", MarkerArray, queue_size=10
        )
        calibrationParams = rospy.wait_for_message("/camera/camera_info", CameraInfo)
        self.dist_coeffs = calibrationParams.D
        self.camera_matrix = np.reshape(calibrationParams.K, (3, 3))
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        if len(corners) > 0:
            marker_array = MarkerArray()
            for i in range(len(corners)):
                ret = cv.aruco.estimatePoseSingleMarkers(
                    corners[i], 0.034, self.camera_matrix, self.dist_coeffs
                )
                (rvec, tvec) = (ret[0][0], ret[1][0])

                # get quaternion for ros. 为ros获取四元数
                euler = rvec[0, :]
                rotation_matrix = np.array(
                    [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]],
                    dtype=float,
                )
                rotation_matrix[:3, :3], _ = cv.Rodrigues(euler)
                # convert the matrix to a quaternion
                quaternion = tf_conversions.transformations.quaternion_from_matrix(
                    rotation_matrix
                )
                marker = Marker()
                marker.header.frame_id = "usb_cam"
                marker.header.stamp = rospy.Time.now()
                marker.id = ids[i][0]
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = tvec[0][0]
                marker.pose.position.y = tvec[0][1]
                marker.pose.position.z = tvec[0][2]
                marker.pose.orientation.x = quaternion[0]
                marker.pose.orientation.y = quaternion[1]
                marker.pose.orientation.z = quaternion[2]
                marker.pose.orientation.w = quaternion[3]
                marker.scale.x = 0.034
                marker.scale.y = 0.034
                marker.scale.z = 0.005
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                marker_array.markers.append(marker)

            self.marker_publisher.publish(marker_array)


if __name__ == "__main__":
    try:
        rospy.init_node("detect_marker")
        rospy.loginfo("Starting detect_marker node")
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down detect_marker node.")
        cv.destroyAllWindows()
