#!/usr/bin/env python3

import rospy
from dviratis_robotas.msg import USB_message
import array
import serial


numOfSensData = 14
sensorPublisher = rospy.Publisher('sensor_raw_data', USB_message, queue_size=10)

serial_port = serial.Serial('/dev/ttyACM0', timeout=0)


def callback(data):
    dataBytes = data.array
    rospy.loginfo(''.join('%02X ' % i for i in data.array))

    if dataBytes[0] == ord('S'):
        # Read sensor data trough usb

        # Send data request byte
        serial_port.write(ord('!'))

        # Wait for mcu to send message
        while serial_port.in_waiting < numOfSensData:
            pass

        mcuPacket = serial_port.read_all()
        if len(mcuPacket) == numOfSensData:
            # Publish received data
            dataToPublish = USB_message()
            dataToPublish.array = mcuPacket
            sensorPublisher.publish(dataToPublish)

    elif dataBytes[0] == ord('M'):
        # Send motor data trough usb
        serial_port.write(dataBytes)

    else:
        # Message unrecognised
        pass


def listener():
    serial_port.flush

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('USB_message', USB_message, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
