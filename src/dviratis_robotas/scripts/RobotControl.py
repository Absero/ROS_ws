#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from dviratis_robotas.msg import USB_message
import tf
from nav_msgs.msg import Odometry
from numpy import uint8

frequency = 10  # Hz
isObjectDetected = False

angle = 0

pub_USB = rospy.Publisher('USB_message', USB_message, queue_size=1)


def updateDetectionResult(msg):
    global isObjectDetected
    isObjectDetected = msg.data > 0


def updateAngle(msg):
    global angle
    roll, pitch, angle = tf.transformations.euler_from_quaternion(
        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])


def calculator():
    rospy.init_node('ObjectRecognition_node', anonymous=True)

    # Apdorojimo rezultatu gavimui
    rospy.Subscriber('DetectionResult_msg', Int32, updateDetectionResult)
    rospy.Subscriber('odom', Odometry, updateAngle)

    msg_USB = USB_message()
    motor_message = [0]*3
    motor_message[0] = ord('M')

    while not rospy.is_shutdown():
        if isObjectDetected:
            value = 0
        else:
            value = 50

        motor_message[1] = uint8(value)
        motor_message[2] = uint8(-value)

        msg_USB.array = motor_message
        pub_USB.publish(msg_USB)

        rospy.Rate(frequency).sleep()


if __name__ == '__main__':
    try:
        calculator()
    except rospy.ROSInterruptException:
        pass
