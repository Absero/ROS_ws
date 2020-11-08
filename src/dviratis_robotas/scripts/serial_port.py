#!/usr/bin/env python

import rospy
from std_msgs.msg import String
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

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        hello_str = ""
        if serial_port.in_waiting > 0:
            dummy = serial_port.read_all()
            hello_str = "%s | %s" % (dummy, len(dummy))
            rospy.loginfo(hello_str)
            pub.publish(hello_str)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
