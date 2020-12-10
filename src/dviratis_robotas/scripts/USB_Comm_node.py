#!/usr/bin/env python3

import rospy
from dviratis_robotas.msg import USB_message
import array
import serial


numOfSensData = 14
sensorPublisher = rospy.Publisher('sensor_raw_data', USB_message, queue_size=10)


def callback(data):
    dataBytes = array.array('B', data.array)

    rospy.loginfo(rospy.get_caller_id() + ' received %s bytes> %s' %
                  (len(data.array), ''.join('%02X ' % i for i in dataBytes)))

    if(dataBytes[0] == 'S'):
        # Read sensor data trough usb

        # Send data request byte
        serial_port.write(b'!')

        # Wait for mcu to send message
        while serial_port.in_waiting < numOfSensData:
            pass

        mcuPacket = serial_port.read_all()
        if len(mcuPacket) == numOfSensData:
            # Publish received data
            dataToPublish = USB_message()
            dataToPublish.array = mcuPacket
            sensorPublisher.publish(dataToPublish)

    elif (dataBytes[0] == 'M'):
        # Send motor data trough usb
        pass

    else:
        # Message unrecognised
        pass


def listener():
    port = '/dev/ttyACM0'

    args = rospy.myargv(argv=sys.argv)
    if len(args) == 2:
        port = args[1]

    serial_port = serial.Serial(port, timeout=0)
    serial_port.flush

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('USB_message', USB_message, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
