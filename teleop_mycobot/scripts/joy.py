import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyControl:
    def __init__(self):
        self._vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._joy_sub = rospy.Subscriber("/joy", Joy, self.joyCallback)

    def joyCallback(self, joy_msg):
        twist = Twist()
        twist.linear.x = -joy_msg.axes[0] * 0.2
        twist.linear.y = joy_msg.axes[1] * 0.2
        twist.angular.z = joy_msg.axes[3] * 1.0

        self._vel_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("joy_controller")
    joy_ctrl = JoyControl()
    rospy.spin()
