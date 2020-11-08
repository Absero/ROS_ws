#!/usr/bin/env python

import rospy
from dviratis_robotas.msg import USB_message
import serial
import sys


def talker():
    port = '/dev/ttyACM0'

    args = rospy.myargv(argv=sys.argv)
    # with rospy.myargv(argv=sys.argv) as args:
    if len(args) == 2:
        port = args[1]

    serial_port = serial.Serial(port, timeout=0)
    serial_port.flush

    pub = rospy.Publisher('USB_message', USB_message, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        if serial_port.in_waiting > 0:
            data = serial_port.read_all()
            rospy.loginfo('Published %s bytes> "%s"' % (len(data), data))
            pub.publish("data")

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
