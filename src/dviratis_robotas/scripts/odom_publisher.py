#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from dviratis_robotas.msg import AccGyr_msg


maxValues = 25  # siuntimo daznis = gavimoDaznis/maxValues

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0

current_time = rospy.Time(0)
last_time = rospy.Time(0)


def publisher(data):
    global x, y, th, vx, vy, current_time, last_time

    current_time = rospy.Time.now()

    vth = data.gyro[2]

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    odom_broadcaster = tf.TransformBroadcaster()

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time


def listener():
    global current_time, last_time
    rospy.init_node('DeadReckoning', anonymous=True)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    rospy.Subscriber('AccGyr_msg', AccGyr_msg, publisher)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
