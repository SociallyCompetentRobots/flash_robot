#!/usr/bin/env python3

from threading import Thread, Event
import time

import cv2

import rospy

from cv_bridge              import CvBridge, CvBridgeError
from geometry_msgs.msg      import Twist
from std_msgs.msg           import Float32, Int16MultiArray, Int16
from sensor_msgs.msg        import Image, LaserScan


from flash_controller.flash import Flash


BRIDGE = CvBridge()


class CamStreamer(Thread):

    URI = 'http://10.0.0.195:8081/video.mjpg'
    FPS = 10


    def __init__(self, event, publisher):
        """ Creates a CaptureData thread given a stop flag, a recording object and a device. """
        Thread.__init__(self)
        self.stopped    = event
        self.pub_image  = publisher

        # in case we do not have a video stream
        try:
            self.cam        = cv2.VideoCapture(CamStreamer.URI)
        except:
            self.stopped.set()


    def run(self):
        """ The run method captures the images and publish them to the topic /flash_robot/cam.
        """

        while self.cam.isOpened() and not self.stopped.wait(1.0 / CamStreamer.FPS):
            ret, frame = self.cam.read()
            if ret:
                self.pub_image.publish(BRIDGE.cv2_to_imgmsg(frame, encoding = "rgb8"))


class FlashNode:
    """ ROS Node to control the flash robot. """

        
    PUBLISHER_RATE  = 30

    
    def __init__(self):
        """ Initializes the FLASH ROS node. """
        self.name = self.__class__.__name__
        rospy.init_node(self.name)

        self.sub_cmd_vel   = rospy.Subscriber('/flash_robot/cmd_vel',     Twist,     self.cmdVelCallback)
        self.sub_behave    = rospy.Subscriber('/flash_robot/behave',      Int16,     self.behaveCallback)
        self.pub_battery   = rospy.Publisher('/flash_robot/battery',      Float32,   queue_size = 1)
        self.pub_laser     = rospy.Publisher('/flash_robot/laser_scan',   LaserScan, queue_size = 1)
        self.pub_image     = rospy.Publisher('/flash_robot/cam',          Image,     queue_size = 1)

        # startup FLASH
        self.flash         = Flash()

        # start camera stream
        self._stop_flag    = Event()
        self.cam_stream    = CamStreamer(self._stop_flag, self.pub_image)
        self.cam_stream.start()

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
        
        # update the battery status
        self.pub_battery.publish(Float32(self.flash.batteryVoltage))
        self.pub_laser.publish(FlashNode.laser2Msg(self.flash.laserValues))

        if self.cmd_vel_flag and (self.cmd_vel_ts - time.time()) > 0.1:
            self.flash.uw.send("robot.body.x.speed = 0 & robot.body.yaw.speed = 0")
            self.cmd_vel_flag = False
            print ("Robot stopped")


    @staticmethod
    def laser2Msg(readings):
        val, ts                 = readings
        val                     = np.array(val, dtype = np.float32)
        val                     = val / 1000.0
        
        laser_frequency         = 30
        num_readings            = 133

        scan = LaserScan()
        scan.header.stamp       = Time.now()
        scan.header.frame_id    = "flash_laser_frame"
        scan.angle_min          = -1.2
        scan.angle_max          =  1.2
        scan.angle_increment    = 3.14 / num_readings
        scan.time_increment     = (1 / laser_frequency) / (num_readings)
        scan.range_min          = 0.0
        scan.range_min          = 5.6
        scan.ranges             = val
        return scan


    def __del__(self):
        self._stop_flag.set()
        self.cam_stream = None
        self._stop_flag = None


if __name__ == '__main__':
    ros_node  = FlashNode()
    ros_rate  = rospy.Rate(ros_node.PUBLISHER_RATE)

    try:
        print(ros_node.name + " started")
        while not rospy.is_shutdown():
            ros_node.update()
            ros_rate.sleep()

    except rospy.ROSInterruptException as exception:
        print (exception)
