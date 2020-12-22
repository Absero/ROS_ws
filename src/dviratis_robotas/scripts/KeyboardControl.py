#!/usr/bin/env python3

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from dviratis_robotas.msg import USB_message
from nav_msgs.msg import Odometry
from numpy import uint8, sign


angle = 0

pub_USB = rospy.Publisher('USB_message', USB_message, queue_size=1)

msg_USB = USB_message()
motor_message = [0]*3
motor_message[0] = ord('M')


def updateAngle(msg):
    global angle
    roll, pitch, angle = tf.transformations.euler_from_quaternion(
        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])


def newCommand(msg):
    angle = msg.angular.z/4     # kampo verte radianais
    forward = msg.linear.x    # tiesiai

    value = (forward - angle) * 100
    value = value if abs(value) < 100 else sign(value)*100
    motor_message[1] = uint8(value)

    value = (forward + angle) * 100
    value = value if abs(value) < 100 else sign(value)*100
    motor_message[2] = uint8(-value)

    rospy.loginfo("m1={}  m2={}  | angle={} x={}".format(
        motor_message[1], motor_message[2], angle, forward))

    msg_USB.array = motor_message
    pub_USB.publish(msg_USB)


def KeyboardControl():
    rospy.init_node('KeyboardControl_node', anonymous=True)

    rospy.Subscriber('odom', Odometry, updateAngle)
    rospy.Subscriber('cmd_vel', Twist, newCommand)

    rospy.spin()


if __name__ == '__main__':
    KeyboardControl()
