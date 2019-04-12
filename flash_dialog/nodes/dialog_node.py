#!/usr/bin/env python3

from threading import Thread, Event
import time

import numpy as np

import rospy



class DialogNode:
    """ ROS Node for dialog management. """

        
    PUBLISHER_RATE  = 30

    
    def __init__(self):
        """ Initializes the FLASH ROS node. """
        self.name = self.__class__.__name__
        rospy.init_node(self.name)


    def update(self):
        pass
        


if __name__ == '__main__':
    ros_node  = DialogNode()
    ros_rate  = rospy.Rate(ros_node.PUBLISHER_RATE)

    try:
        rospy.loginfo(ros_node.name + " started")
        while not rospy.is_shutdown():
            ros_node.update()
            ros_rate.sleep()

    except rospy.ROSInterruptException as exception:
        print (exception)
