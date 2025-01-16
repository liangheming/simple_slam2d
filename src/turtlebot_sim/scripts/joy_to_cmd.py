#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyToCmd:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.max_vel = rospy.get_param("~max_vel", 1.0)
        self.max_rot = rospy.get_param("~max_rot", 1.5)

    def joy_callback(self, joy_msg):
        vel_cmd = joy_msg.axes[1] * self.max_vel
        rot_cmd = joy_msg.axes[3] * self.max_rot
        cmd_msg = Twist()
        cmd_msg.linear.x = vel_cmd
        cmd_msg.angular.z = rot_cmd
        self.cmd_vel_pub.publish(cmd_msg)


if __name__ == "__main__":
    rospy.init_node("joy_to_cmd", anonymous=True)
    joy_to_cmd = JoyToCmd()
    rospy.spin()
