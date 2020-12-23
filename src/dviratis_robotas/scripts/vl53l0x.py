#!/usr/bin/python3

import rospy
import VL53L0X
from sensor_msgs.msg import LaserScan
from numpy import float32


pub_scan = rospy.Publisher('scan', LaserScan, queue_size=10)

# Create a VL53L0X object
tof = VL53L0X.VL53L0X(i2c_bus=1, i2c_address=0x29)
tof.open()

# Start ranging
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.LONG_RANGE)

current_time = rospy.Time(0)
last_time = rospy.Time(0)


def main():
    rospy.init_node('VL53L0X_node', anonymous=True)

    timing = tof.get_timing()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        distance = tof.get_distance()  # Nuskaityti duomenis
        rospy.loginfo("dist={}".format(distance))

        # Atmesti negerus duomenis
        if distance > 3000 or distance < 0:
            distance = 0

        # Headeris
        laser_scan = LaserScan()
        laser_scan.header.stamp = current_time
        laser_scan.header.frame_id = "radar"

        laser_scan.angle_min = -0.15
        laser_scan.angle_max = 0.15
        laser_scan.angle_increment = 0.05
        laser_scan.time_increment = 0
        laser_scan.scan_time = timing
        laser_scan.range_min = 0.030
        laser_scan.range_max = 2.500
        laser_scan.ranges = [float32(distance/1000)]*7

        pub_scan.publish(laser_scan)

        rospy.Rate(1/(timing/1000000.00)).sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        tof.stop_ranging()
        tof.close()
