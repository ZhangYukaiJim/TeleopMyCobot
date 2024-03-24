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
        self.load_camera_info()
        self.cap = cv2.VideoCapture(
            "udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER,
        )
        if not self.cap.isOpened():
            rospy.logerr("Failed to open MJPEG stream")
            rospy.signal_shutdown("Failed to open MJPEG stream")
            return

    def load_camera_info(self):
        with open(rospy.get_param("~camera_info_file"), "r") as f:
            self.camera_info = yaml.safe_load(f)

    def publish_image(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                ros_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image_msg.header.frame_id = "usb_cam"
                self.image_pub.publish(ros_image_msg)
                self.publish_camera_info()
            else:
                rospy.logwarn("Failed to read frame from MJPEG stream")
                break

    def publish_camera_info(self):
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
        camera_publisher = CameraPublisher()
        camera_publisher.publish_image()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
        camera_publisher.cap.release()
