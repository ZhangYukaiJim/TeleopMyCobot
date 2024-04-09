import rospy
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from p5 import *
from PIL import Image as PILImage


bridge = CvBridge()


def setup():
    size(1280, 720)  # create window: width, height


def draw():
    pass  # we will update the screen in the callback


def callback(data):
    # Convert the image from ROS to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # Convert the OpenCV image to a PIL image
    pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

    # Draw the image
    image(pil_image, 0, 0)


def listener():
    rospy.Subscriber("/camera/image_raw", Image, callback)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("image_listener", anonymous=True)
    listener_thread = threading.Thread(target=listener)
    listener_thread.start()
    run()
