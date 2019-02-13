#! /usr/bin/env python

import rospy

import actionlib

from flash_controller.flash import Flash
from flash_behaviors.msg import ActAction


class ActionServer(object):
    
    def __init__(self, name):
        
        # Robot controller.
        self.flash = Flash(rospy.get_param("~filename"))

        # Action server.
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ActAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        rospy.loginfo("Action server started!")

      
    def execute_cb(self, goal):
        
        try:
            self.flash.uw.send(goal.action)
        except Exception as e:
            print(e)
        
        self._as.set_succeeded()
        

if __name__ == '__main__':
    
    rospy.init_node('action_server')
    server = ActionServer(rospy.get_name())
    rospy.spin()