#!/usr/bin/env python3

import numpy
import rospy
import sensor_msgs.msg

from flash_controller.laser import Laser

# Constants for LaserScan message
RATE            =  20
NUM_READINGS    =  133
FRAME_ID        = "flash_robot_laser_link"


def scan2ROS(val):
    """ Convert the laser scan readings into a ROS LaserScan message. 

        FLASH sends the distance as mm hence the conversion into meters. The message will report
        Be aware: ROS time instead of FLASH timestamp!
    """
    val                     = numpy.array(val, dtype = numpy.float32) / 1000.0
    scan                    = sensor_msgs.msg.LaserScan()
    scan.header.stamp       = rospy.Time.now()
    scan.header.frame_id    = FRAME_ID
    scan.angle_min          = -1.2
    scan.angle_max          =  1.2
    scan.angle_increment    = 3.1415 / NUM_READINGS
    scan.time_increment     = (1. / RATE) / (NUM_READINGS)
    scan.scan_time          = 1. / RATE
    scan.range_min          = 0.0
    scan.range_max          = 5.6
    scan.ranges             = val
    return scan


def main():
    rospy.init_node('LaserNode')

    laser    = Laser()
    pub      = rospy.Publisher('/flash_robot/laser_scan', sensor_msgs.msg.LaserScan, queue_size = 1)
    ros_rate = rospy.Rate(RATE)

    rospy.loginfo("LaserNode started")

    # keep going till shutdown
    while not rospy.is_shutdown():
        
        # read the values from FLASH
        scan = laser.scan
        
        # scan might be None if we had a communication issue reading the laser scan from FLASH
        if scan:
            pub.publish(scan2ROS(scan[0]))

        ros_rate.sleep()


if __name__ == '__main__':
    main()
