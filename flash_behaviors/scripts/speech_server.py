#! /usr/bin/env python

import rospy

import actionlib

from flash_controller.flash import Flash
from flash_behaviors.msg import SpeakAction

class SpeakActionServer(object):
    
    def __init__(self, name):
        
        # Robot controller.
        self.flash = Flash()

        # Action server.
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, SpeakAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        rospy.loginfo("Speech action server started!")

      
    def execute_cb(self, goal):
        
        text = goal.text
        duration = goal.duration

        if text == '':
            rospy.loginfo("Invalid request. The text field was empty!")
            self._as.set_rejected()
        else:
            if duration:
                self.flash.say(text, duration)
            else:
                self.flash.say(text)
            
            self._as.set_succeeded()

        
if __name__ == '__main__':
    
    rospy.init_node('speech_server')
    server = SpeakActionServer(rospy.get_name())
    rospy.spin()