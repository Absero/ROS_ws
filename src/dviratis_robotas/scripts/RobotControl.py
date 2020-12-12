#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import tf
from nav_msgs.msg import Odometry

frequency = 10  # Hz
isObjectDetected = False

angle = 0


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

    while not rospy.is_shutdown():
        rospy.loginfo("Object detected: {}".format(isObjectDetected))
        rospy.Rate(frequency).sleep()


if __name__ == '__main__':
    try:
        calculator()
    except rospy.ROSInterruptException:
        pass
