#!/usr/bin/env python3

import numpy
import rospy
import sensor_msgs.msg

from flash_controller.laser import Laser


def scan2ROS(val):
    val                     = numpy.array(val, dtype = numpy.float32)
    val                     = val / 1000.0
    
    laser_frequency         = 30
    num_readings            = 133

    scan                    = sensor_msgs.msg.LaserScan()
    scan.header.stamp       = rospy.Time.now()                # we use the ROS time for the message!
    scan.header.frame_id    = "flash_robot_laser_link"
    scan.angle_min          = -1.2
    scan.angle_max          =  1.2
    scan.angle_increment    = 3.14 / num_readings
    scan.time_increment     = (1 / laser_frequency) / (num_readings)
    scan.range_min          = 0.0
    scan.range_min          = 5.6
    scan.ranges             = val
    return scan


def main():
    rospy.init_node('LaserNode')

    laser    = Laser()
    pub      = rospy.Publisher('/flash_robot/laser_scan', sensor_msgs.msg.LaserScan, queue_size = 1)
    ros_rate = rospy.Rate(30)

    print("LaserNode started")
    while not rospy.is_shutdown():
        scan = laser.scan
        if scan:
            pub.publish(scan2ROS(scan[0]))
        ros_rate.sleep()

if __name__ == '__main__':
    main()