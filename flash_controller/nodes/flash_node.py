#!/usr/bin/env python3

from threading import Thread, Event
import time

import numpy as np

import rospy

from geometry_msgs.msg      import Twist
from std_msgs.msg           import Float32, Int16MultiArray, Int16
from sensor_msgs.msg        import Image


from flash_controller.flash import Flash


class FlashNode:
    """ ROS Node to control the flash robot. """

        
    PUBLISHER_RATE  = 30

    
    def __init__(self):
        """ Initializes the FLASH ROS node. """
        self.name = self.__class__.__name__
        rospy.init_node(self.name)

        self.sub_cmd_vel   = rospy.Subscriber('/flash_robot/cmd_vel',     Twist,     self.cmdVelCallback)
        self.sub_behave    = rospy.Subscriber('/flash_robot/behave',      Int16,     self.behaveCallback)

        # startup FLASH
        self.flash         = Flash()

        self.cmd_vel_ts    = time.time()
        self.cmd_vel_flag  = False


    def cmdVelCallback(self, msg):
        self.cmd_vel_ts   = time.time()
        self.cmd_vel_flag = True
        cmd               = "robot.body.x.speed = %i & robot.body.yaw.speed = %i" % (msg.linear.x, msg.angular.z)
        self.flash.uw.send(cmd)
        print('cmd_vel', cmd)


    def behaveCallback(self, msg):
        if msg.data == 1:
            self.flash.say('Hello. My name is Alyx. Nice to meet you. Welcome to the HRI Laboratory.')
            self.flash.say('You might have noticed that I have a very expressive face.')
            self.flash.say('I can get angry.')
            self.flash.exp('Angry')
            self.flash.say('I can show disgust.')
            self.flash.exp('Disgust')
            self.flash.say('Also, I can be sad.')
            self.flash.exp('Sad')
            self.flash.say('or be surprised.')
            self.flash.exp('Surprise')
            self.flash.say('and sometimes I am afraid')
            self.flash.exp('Fear')
            self.flash.say('If I am bored I can get very sleepy')
            self.flash.exp('Yawn')


    def update(self):
        
        if self.cmd_vel_flag and (self.cmd_vel_ts - time.time()) > 0.1:
            self.flash.uw.send("robot.body.x.speed = 0 & robot.body.yaw.speed = 0")
            self.cmd_vel_flag = False
            print ("Robot stopped")


if __name__ == '__main__':
    ros_node  = FlashNode()
    ros_rate  = rospy.Rate(ros_node.PUBLISHER_RATE)

    try:
        rospy.loginfo(ros_node.name + " started")
        while not rospy.is_shutdown():
            ros_node.update()
            ros_rate.sleep()

    except rospy.ROSInterruptException as exception:
        print (exception)
