#!/usr/bin/env python3

from threading import Thread, Event

import cv2

import rospy

from cv_bridge          import CvBridge, CvBridgeError
from geometry_msgs.msg  import Twist
from std_msgs.msg       import Float32, Int16MultiArray
from sensor_msgs.msg    import Image, LaserScan


from flash.flash        import Flash


fps_time    = 0
bridge      = CvBridge()


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
                self.pub_image.publish(bridge.cv2_to_imgmsg(frame, encoding = "rgb8"))


class FlashNode(RosNode):
    """ ROS Node to control the flash robot. """

        
    PUBLISHER_RATE  = 30

    
    def __init__(self):
        """ Initializes the FLASH ROS node. """
        self.name = self.__class__.__name__
        rospy.init_node(self.name)

        self.sub_cmd_vel   = rospy.Subscriber('/flash_robot/cmd_vel',     Twist,     self.cmdVelCallback, (self, ))
        self.pub_battery   = rospy.Publisher('/flash_robot/battery',      Float32,   queue_size = 1)
        self.pub_laser     = rospy.Publisher('/flash_robot/laser_scan',   LaserScan, queue_size = 1)
        self.pub_image     = rospy.Publisher('/flash_robot/cam',          Image,     queue_size = 1)

        # startup FLASH
        self.flash         = Flash()

        # start camera stream
        self._stop_flag    = Event()
        self.cam_stream    = CamStreamer(self._stop_flag)
        self.cam_stream.start()


    def cmdVelCallback(self):
        
        if not hasattr(self, 'flash'):
            return
        
        if self.sub_cmd_vel.message_list:
            cmd_vel  = self.sub_cmd_vel.message_list[0]
            print(cmd_vel)
#            self.flash.translateAndRotate(0.1, cmd_vel.linear.x, cmd_vel.angular.z)


    def update(self):
        
        # update the battery status
        self.pub_battery.publish(Float32(self.flash.batteryVoltage))
        self.pub_laser.publish(FlashNode.laser2Msg(self.flash.laserValues))


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
        ros_node.logdebug(ros_node.name + " started")
        while not rospy.is_shutdown():
            ros_node.update()
            ros_rate.sleep()

    except rospy.ROSInterruptException as exception:
        print (exception)
