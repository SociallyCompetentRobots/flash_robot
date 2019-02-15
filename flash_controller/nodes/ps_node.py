#!/usr/bin/env python3
import psutil

import rospy
from std_msgs.msg import Bool

RATE = 5

def main():
    rospy.init_node('PsNode')
    proc_name = './test.sh'
    pub       = rospy.Publisher('/system/ps/psychopy', Bool, queue_size = 1)
    ros_rate  = rospy.Rate(RATE)

    rospy.loginfo("PsNode started")

    # keep going till shutdown
    while not rospy.is_shutdown():

        for pid in psutil.pids():
            try:
                if psutil.Process(pid).cmdline()[1] == proc_name:                    
                    pub.publish(Bool(True))
                    break
            except:
                pass
        ros_rate.sleep()

if __name__ == '__main__':
    main()
