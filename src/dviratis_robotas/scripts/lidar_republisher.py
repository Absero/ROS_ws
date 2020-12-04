#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

pub = rospy.Publisher('scan', LaserScan, queue_size=10)


def callback(data):
    # change timestamp and publish
    pradinis = data.header.stamp
    data.header.stamp = rospy.Time.now()

    pub.publish(data)

    # rospy.loginfo("Scan {} => {}".format(pradinis, data.header.stamp))


def listener():
    rospy.init_node('scan_republisher', anonymous=True)

    rospy.Subscriber('/scan1', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
