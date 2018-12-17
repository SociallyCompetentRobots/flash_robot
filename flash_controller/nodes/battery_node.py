#!/usr/bin/env python3

import rospy
import std_msgs.msg

from flash_controller.battery import Battery

RATE = 10


def main():
    rospy.init_node('BatteryNode')

    battery  = Battery()
    pub      = rospy.Publisher('/flash_robot/battery', std_msgs.msg.Float32, queue_size = 1)
    ros_rate = rospy.Rate(RATE)

    rospy.loginfo("BatteryNode started")

    # keep going till shutdown
    while not rospy.is_shutdown():
        # voltage ranges from 32. to 42.
        voltage          = battery.voltage
        percentage_level = (voltage - 32.)*10.
        pub.publish(std_msgs.msg.Float32(percentage_level))
        ros_rate.sleep()


if __name__ == '__main__':
    main()
