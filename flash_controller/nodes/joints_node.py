#!/usr/bin/env python3

import rospy
from std_msgs.msg           import Float32MultiArray, MultiArrayDimension

from flash_controller.flash_state import FlashState


RATE = 20

def states2ROS(val):
    """ Convert the joints state readings into a ROS Float32MultiArray message. 

        Be aware: ROS time instead of FLASH timestamp!
    """
    mat = Float32MultiArray()
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim[0].label     = 'joint states'
    mat.layout.dim[0].size      = len(val)
    mat.layout.dim[0].stride    = 1
    mat.layout.data_offset      = 0
    mat.data                    = val
    return mat


def main():
    rospy.init_node('JointsNode')

    flash_state = FlashState()
    pub         = rospy.Publisher('/flash_robot/joints_pos', Float32MultiArray, queue_size = 1)
    ros_rate    = rospy.Rate(RATE)

    rospy.loginfo("JointsNode started")

    # keep going till shutdown
    while not rospy.is_shutdown():
        pub.publish( states2ROS( [joint.pos for joint in flash_state.joints] ) )
        ros_rate.sleep()


if __name__ == '__main__':
    main()
