#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from dviratis_robotas.msg import AccGyr_msg

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0

ax_old = 0.0
ay_old = 0.0
ax_old_filt = 0.0
ay_old_filt = 0.0

current_time = rospy.Time(0)
last_time = rospy.Time(0)


def publisher(data):
    global x, y, th, vx, vy, current_time, last_time, ax_old, ay_old, ax_old_filt, ay_old_filt

    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    # naujas kampas
    vth = data.gyro[2]
    vth = 0
    delta_th = vth * dt
    th += delta_th

    # nauji pagreiciai
    ax = data.acc[0]
    ay = data.acc[1]

    # perskaiciuoti kiekvienai asiai
    ax_real = (ax * cos(th) - ay * sin(th)) * dt
    ay_real = (ax * sin(th) + ay * cos(th)) * dt

    # filtruoti
    R = 0.995
    ax = ax_real - ax_old + R * ax_old_filt
    ay = ay_real - ay_old + R * ay_old_filt

    # issaugoti reiksmes
    ax_old = ax_real
    ay_old = ay_real

    ax_old_filt = ax
    ay_old_filt = ay

    # integruoti reiksmes
    vx += ax_real
    vy += ay_real

    rospy.loginfo('{: 04.4f} | {: 04.4f} | {: 04.4f}'.format(
        vx, ax, th))

    # compute odometry in a typical way given the velocities of the robot
    delta_x = vx * dt
    delta_y = vy * dt

    x += delta_x
    y += delta_y

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

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
