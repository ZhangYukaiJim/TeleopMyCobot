import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher:
    def __init__(self):
        rospy.init_node("camera_publisher", anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)

        self.cap = cv2.VideoCapture(
            "udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER,
        )
        if not self.cap.isOpened():
            rospy.logerr("Failed to open MJPEG stream")
            rospy.signal_shutdown("Failed to open MJPEG stream")
            return

    def publish_image(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                ros_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image_msg.header.frame_id = "usb_cam"
                self.image_pub.publish(ros_image_msg)
            else:
                rospy.logwarn("Failed to read frame from MJPEG stream")
                break


if __name__ == "__main__":
    try:
        camera_publisher = CameraPublisher()
        camera_publisher.publish_image()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
        camera_publisher.cap.release()
