#!/usr/bin/env python3

from threading import Thread, Event
import time

import numpy as np

import rospy

from geometry_msgs.msg      import Twist
from std_msgs.msg           import Float32, Int16MultiArray, Int16
from sensor_msgs.msg        import Image


from flash_controller.head  import Head


class HeadNode:
    """ ROS Node to control the flash robot. """

        
    PUBLISHER_RATE  = 30

    
    def __init__(self):
        """ Initializes the FLASH ROS node. """
        self.name = self.__class__.__name__
        rospy.init_node(self.name)

        self.sub_cmd_vel   = rospy.Subscriber('/flash_robot/head/cmd_vel', Twist, self.cmdVelCallback)
        self.sub_behave    = rospy.Subscriber('/flash_robot/head/behave',  Int16, self.behaveCallback)

        # startup EMYS
        self.head          = Head()


    def cmdVelCallback(self, msg):
        j_yaw     = self.head.head_yaw
        j_pitch   = self.head.neck_pitch
        pos_yaw   = j_yaw.clipRawLimits(j_yaw.value + msg.angular.y)
        pos_pitch = j_yaw.clipRawLimits(j_yaw.value + msg.angular.z)
        cmd       = "%s.val = %.2f &" % (j_yaw.name,   pos_yaw)
        cmd      += "%s.val = %.2f"   % (j_pitch.name, pos_pitch)
        self.head.uw.send(cmd)


    def behaveCallback(self, msg):
        pass


    def update(self):
        pass


if __name__ == '__main__':
    ros_node  = HeadNode()
    ros_rate  = rospy.Rate(ros_node.PUBLISHER_RATE)

    try:
        rospy.loginfo(ros_node.name + " started")
        while not rospy.is_shutdown():
            ros_node.update()
            ros_rate.sleep()

    except rospy.ROSInterruptException as exception:
        print (exception)
