#!/usr/bin/env python3

import rospy
from dviratis_robotas.msg import USB_message
import array


def callback(data):
    dataBytes = array.array('B', data.array)
    # dataBytes = data.array

    rospy.loginfo(rospy.get_caller_id() + ' received %s bytes> %s' %
                  (len(data.array), ''.join('%02X ' % i for i in dataBytes)))


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('USB_message', USB_message, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
