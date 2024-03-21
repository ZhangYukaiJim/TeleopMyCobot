#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformBroadcaster
import tf_conversions
import geometry_msgs
# from mycobot_communication.srv import (
#     GetCoords,
#     SetCoords,
#     GetAngles,
#     SetAngles,
#     GripperStatus,
# )


class ImageConverter:
    def __init__(self):
        self.br = TransformBroadcaster()
        self.bridge = CvBridge()
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        self.parameters = cv.aruco.DetectorParameters()
        calibrationParams = rospy.wait_for_message('/camera/camera_info', CameraInfo)
        rospy.loginfo(calibrationParams)
        self.dist_coeffs = calibrationParams.getNode("distCoeffs").mat()
        self.camera_matrix = None
        # subscriber, listen wether has img come in. 订阅者，监听是否有img
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.callback)

    def callback(self, data):
        """Callback function.

        Process image with OpenCV, detect Mark to get the pose. Then acccording the
        pose to transforming.
        """
        try:
            # trans `rgb` to `gbr` for opencv. 将 `rgb` 转换为 opencv 的 `gbr`。
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        size = cv_image.shape
        focal_length = size[1]
        center = [size[1] / 2, size[0] / 2]
        if self.camera_matrix is None:
            # calc the camera matrix, if don't have.如果没有，则计算相机矩阵
            self.camera_matrix = np.array(
                [
                    [focal_length, 0, center[0]],
                    [0, focal_length, center[1]],
                    [0, 0, 1],
                ],
                dtype=np.float32,
            )
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        # detect aruco marker.检测 aruco 标记
        ret = cv.detectMarkers(gray, self.dictionary, self.parameters)
        corners, ids = ret[0], ret[1]
        # process marker data.处理标记数据
        if len(corners) > 0:
            if ids is not None:
                # print('corners:', corners, 'ids:', ids)

                # detect marker pose. 检测marker位姿。
                # argument:
                #   marker corners,标记角
                #   marker size (meter),标记尺寸（米）
                ret = cv.aruco.estimatePoseSingleMarkers(
                    corners, 0.05, self.camera_matrix, self.dist_coeffs
                )
                (rvec, tvec) = (ret[0], ret[1])
                (rvec - tvec).any()

                print("rvec:", rvec, "tvec:", tvec)

                # just select first one detected marker.只需选择第一个检测到的标记。
                for i in range(rvec.shape[0]):
                    cv.aruco.drawDetectedMarkers(cv_image, corners)
                    cv.drawFrameAxes(
                        cv_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec[i, :, :],
                        tvec[i, :, :],
                        0.03,
                    )

                xyz = tvec[0, 0, :]
                xyz = [xyz[0] - 0.045, xyz[1], xyz[2] - 0.03]

                # get quaternion for ros. 为ros获取四元数
                euler = rvec[0, 0, :]
                tf_change = tf_conversions.transformations.quaternion_from_euler(
                    euler[0], euler[1], euler[2]
                )
                print("tf_change:", tf_change)

                # trans pose according [joint1]，根据 [joint1] 变换姿势
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now() 
                t.header.frame_id = "joint1"
                t.child_frame_id = 'basic_shape'
                t.transform.translation.x = xyz[0] 
                t.transform.translation.y = xyz[1]
                t.transform.translation.z = xyz[2]
                t.transform.rotation.x = tf_change[0]
                t.transform.rotation.y = tf_change[1]
                t.transform.rotation.z = tf_change[2]
                t.transform.rotation.w = tf_change[3]
                self.br.sendTransform(t)

        # [x, y, z, -172, 3, -46.8]
        cv.imshow("Image", cv_image)

        cv.waitKey(3)
        try:
            pass
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    try:
        rospy.init_node("detect_marker")
        rospy.loginfo("Starting cv_bridge_test node")
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv.destroyAllWindows()
