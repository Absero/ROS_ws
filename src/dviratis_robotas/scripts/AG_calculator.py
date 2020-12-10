#!/usr/bin/env python3

import rospy
from dviratis_robotas.msg import AccGyr_msg
from dviratis_robotas.msg import USB_message
import serial
import sys
from numpy import int16, uint8
from math import pi

# settings
accFS = 8  # 2 4 8 16
gyroFS = 1000   # 250 500 1000 2000
frequency = 100  # nuskaitymo daznis

acc_values = [0]*3
gyr_values = [0]*3

offset_x = 0
offset_y = 0

pub_AG = rospy.Publisher('AccGyr_msg', AccGyr_msg, queue_size=10)
pub_USB = rospy.Publisher('USB_message', USB_message, queue_size=10)

mAccGyrDataToPublish = AccGyr_msg()


def callback(data):
    global offset_x, offset_y

    array = data.array
    if(len(array) == 14):
        for i in range(3):
            acc_values[i] = int16(((array[2*i] & 0xff) << 8) | (array[2*i+1] & 0xff)) * \
                accFS / 32768 * 9.8
            gyr_values[i] = int16(((array[2*i+8] & 0xff) << 8) | (array[2*i+9] & 0xff)) * \
                gyroFS / 32768 * pi/180

        if offset_x == 0 and offset_y == 0:
            offset_x = acc_values[0]
            offset_y = acc_values[1]

        acc_values[0] = acc_values[0]-offset_x
        acc_values[1] = acc_values[1]-offset_y

        mAccGyrDataToPublish.acc = acc_values
        mAccGyrDataToPublish.gyro = gyr_values

        rospy.loginfo('acc: {:04.4f} {:04.4f} {:04.4f} gyro: {:04.4f} {:04.4f} {:04.4f}'.format(
            acc_values[0], acc_values[1], acc_values[2], gyr_values[0], gyr_values[1], gyr_values[2]))

        # publish message
        pub_AG.publish(mAccGyrDataToPublish)


def talker():
    rospy.init_node('acc_gyro_calculator', anonymous=True)

    rospy.Subscriber('sensor_raw_data', USB_message, callback)

    rate = rospy.Rate(frequency)  # x hz

    periodicDataPacket = USB_message()
    periodicDataPacket.array = [uint8(83)]

    while not rospy.is_shutdown():
        pub_USB.publish(periodicDataPacket)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
