import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml


class CameraPublisher:
    def __init__(self):
        rospy.init_node("camera_publisher", anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher(
            "/camera/camera_info", CameraInfo, queue_size=10
        )

        self.cap = cv2.VideoCapture(
            "udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER,
        )
        if not self.cap.isOpened():
            rospy.logerr("Failed to open MJPEG stream")
            rospy.signal_shutdown("Failed to open MJPEG stream")
            return

        camera_info_file = rospy.get_param("~camera_info_file", None)
        if camera_info_file is not None:
            self.load_camera_info(camera_info_file)
            self.camera_info_loaded = True
        else:
            rospy.logerr(
                "No camera_info_file parameter specified. CameraInfo publisher will not start."
            )
            self.camera_info_loaded = False

    def load_camera_info(self, camera_info_file):
        with open(camera_info_file, "r") as f:
            self.camera_info = yaml.safe_load(f)

    def publish_image_and_info(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                ros_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image_msg.header.frame_id = "usb_cam"
                timestamp = rospy.Time.now()  # Get current time stamp
                ros_image_msg.header.stamp = (
                    timestamp  # Set timestamp for Image message
                )
                self.image_pub.publish(ros_image_msg)

                if self.camera_info_loaded:
                    camera_info_msg = CameraInfo()
                    camera_info_msg.header.stamp = (
                        timestamp  # Set timestamp for CameraInfo message
                    )
                    camera_info_msg.header.frame_id = (
                        "camera_frame"  # Set the frame ID here
                    )
                    camera_info_msg.width = self.camera_info["image_width"]
                    camera_info_msg.height = self.camera_info["image_height"]
                    camera_info_msg.K = self.camera_info["camera_matrix"]["data"]
                    camera_info_msg.D = self.camera_info["distortion_coefficients"][
                        "data"
                    ]
                    camera_info_msg.R = self.camera_info["rectification_matrix"]["data"]
                    camera_info_msg.P = self.camera_info["projection_matrix"]["data"]
                    self.camera_info_pub.publish(camera_info_msg)
            else:
                rospy.logwarn("Failed to read frame from MJPEG stream")
                break


if __name__ == "__main__":
    try:
        camera_publisher = CameraPublisher()
        camera_publisher.publish_image_and_info()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
        camera_publisher.cap.release()
