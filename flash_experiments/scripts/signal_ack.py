#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from sound_play.msg import SoundRequest


class SoundPlayClient:

    def __init__(self):

        self.process_status = False

        # Process signal subscriber.
        self.process_sub = rospy.Subscriber('/system/ps/psychopy', Bool, self.process_cb)

        self.sound_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=1)

        rospy.loginfo("Signal acknowledgment node initialized...")


    # Subscriber callback.
    def process_cb(self, data):
        
        self.process_status = data.data

        if self.process_status:

            rospy.loginfo("Signal received!")

            self.play_sound()
            rospy.signal_shutdown('All done!')


    def play_sound(self):

        req = SoundRequest()

        req.sound = 3
        req.command = 1
        req.volume = 1.

        self.sound_pub.publish(req)


if __name__ == '__main__':

    rospy.init_node('signal_acknowledgment_node')

    spc = SoundPlayClient()

    rospy.spin()