#!/usr/bin/env python3

import rospy
from dviratis_robotas.msg import AccGyr_msg
import serial
import sys
from numpy import int16

# settings
accFS = 8  # 2 4 8 16
gyroFS = 1000   # 250 500 1000 2000
frequency = 25  # nuskaitymo daznis


def talker():
    port = '/dev/ttyACM0'
    acc_values = [0]*3
    gyr_values = [0]*3

    args = rospy.myargv(argv=sys.argv)
    if len(args) == 2:
        port = args[1]

    serial_port = serial.Serial(port, timeout=0)
    serial_port.flush

    pub = rospy.Publisher('AccGyr_msg', AccGyr_msg, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(frequency)  # x hz

    dataToPublish = AccGyr_msg()

    while not rospy.is_shutdown():
        serial_port.write(b'!')
        rospy.sleep(0.001)

        if serial_port.in_waiting > 0:
            array = serial_port.read_all()
            if(len(array) == 14):
                for i in range(3):
                    acc_values[i] = int16(((array[2*i] & 0xff) << 8) | (array[2*i+1] & 0xff)) * \
                        accFS / 32768 * 9.8
                    gyr_values[i] = int16(((array[2*i+8] & 0xff) << 8) | (array[2*i+9] & 0xff)) * \
                        gyroFS / 32768 * 3.14/180

                dataToPublish.acc = acc_values
                dataToPublish.gyro = gyr_values

                rospy.loginfo('acc: {:04.4f} {:04.4f} {:04.4f} gyro: {:04.4f} {:04.4f} {:04.4f}'.format(
                    acc_values[0], acc_values[1], acc_values[2], gyr_values[0], gyr_values[1], gyr_values[2]))

                # publish message
                pub.publish(dataToPublish)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
