import time

from flash_controller.urbi_wrapper import UrbiWrapper


class Flash:
    """ The Flash class provides the Python API for the FLASH robot. """
    
    DEF_SPEED_ROTATION    = 10
    DEF_SPEED_TRANSLATION = 100

    MAX_SPEED_ROTATION    = 20
    MAX_SPEED_TRANSLATION = 200

    
    def __init__(self):
        """ Creates an Flash API object and connects to the robot. """
        self.uw     = UrbiWrapper()
        value, _   = self.uw.send('4*9')

        # check if the result from the dummy request fits; otherwise abort
        if int(value) != 36:
            self.uw = None
            raise RuntimeError('Connection to Flash failed with result: %s' % value)


    def say(self, text, duration = 2):
        """ Speaks the given text for the given duration. The duration is not yet calculated 
            based on the text.
        """
        self.uw.send('robot.body.neck.head.Say("%s", %i, 0)' % (text, duration))
        time.sleep(duration)


    def translate(self, duration, speed = DEF_SPEED_TRANSLATION):
        """ Moves the robot forward if the speed is positive and backwards for negative speed. 
        
        The current speed limit is 200mm/s and the robot will stop the movement afterwards.
        @param duration - amount of seconds the robot will keep going before stopping
        @param speed    - given in mm/s (default: 100)
        """
        if speed != 0:
            speed = max(min(speed, Flash.MAX_SPEED_TRANSLATION), -Flash.MAX_SPEED_TRANSLATION)
            self.uw.send("robot.body.x.speed = %i" % speed)
            time.sleep(duration)
        self.uw.send("robot.body.x.speed = 0")
        

    def rotate(self, duration, speed = DEF_SPEED_ROTATION):
        """ Rotates the robot left if the speed is positive and right for negative speed. 
        
        The current speed limit is 20deg/s and the robot will stop the movement afterwards.
        @param duration - amount of seconds the robot will keep going before stopping
        @param speed    - given in deg/s (default: 10)
        """
        if speed != 0:
            speed = max(min(speed, Flash.MAX_SPEED_ROTATION), -Flash.MAX_SPEED_ROTATION )
            self.uw.send("robot.body.yaw.speed = %i" % speed)
            time.sleep(duration)    
        self.uw.send("robot.body.yaw.speed = 0")


    def translateAndRotate(self, duration, x_speed = DEF_SPEED_TRANSLATION, yaw_speed = DEF_SPEED_ROTATION):
        """ Moves and rotates the robot at the same time as one action.
        
        The current speed limit is 200mm/s for translation and 20deg/s for rotation. The robot will
        stop the movement afterwards.
        @param duration  - amount of seconds the robot will keep going before stopping
        @param x_speed   - given in mm/s     (default: 100)
        @param yaw_speed - given in deg/s    (default: 10)
        """
        x_speed     = max(min(x_speed,   Flash.MAX_SPEED_TRANSLATION), -Flash.MAX_SPEED_TRANSLATION)
        yaw_speed   = max(min(yaw_speed, Flash.MAX_SPEED_ROTATION),    -Flash.MAX_SPEED_ROTATION   )
        self.uw.send("robot.body.x.speed = %i & robot.body.yaw.speed = %i" % (x_speed, yaw_speed))
        time.sleep(duration)
        self.uw.send("robot.body.x.speed = 0 &  robot.body.yaw.speed = 0")


    def stop(self):
        """ Stops the robot from moving around and brings it in the default position. """
        self.translate(0, 0)
        self.rotate(0, 0)
        self.uw.send("robot.body.neck.head.BehaveNormal(2)")
        self.uw.send("robot.body.arm.MoveCenterDown2(5)")
        time.sleep(7)


    def forward(self, duration, speed = DEF_SPEED_TRANSLATION):
        """ Convenience method for moving forward (@see translate). """
        self.translate(duration, speed)


    def backward(self, duration, speed = DEF_SPEED_TRANSLATION):
        """ Convenience method for moving backwards (@see translate). """
        self.translate(duration, -speed)
    

    def turnLeft(self, duration, speed = DEF_SPEED_ROTATION):
        """ Convenience method for rotating left (@see rotate). """
        self.rotate(duration, speed)


    def turnRight(self, duration, speed = DEF_SPEED_ROTATION):
        """ Convenience method for rotating right (@see rotate). """
        self.rotate(duration, -speed)
    

    def triggerBehavior(self, behavior):
        self.uw.send('robot.competency.%s()' % behavior)


    def exp(self, behavior, duration = 2, intensity = 8):
        if behavior == 'Yawn':
            cmd = 'robot.body.neck.head.BehaveYawn(%s)' % duration
        else:
            cmd = 'robot.body.neck.head.Behave%s(%s, %s)' % (behavior, intensity, duration)
        self.uw.send(cmd)
        time.sleep(duration)
        self.stop()

    def backgroundBehavior(self, behavior, duration = 3, intensity = 8):
        self.uw.send("robot.body.neck.head.Act%s(%s, %s)" % (behavior, intensity, duration))


    @property
    def batteryVoltage(self):
        return float(self.uw.send("robot.body.battery.voltage")[0])


    @property
    def laserValues(self):
        reading, ts = self.uw.send("robot.body.laser.val")
        return eval(reading), ts




    def __del__(self):
        """ Reseting the robot before closing the object. """
        if self.uw is not None:
            self.stop()
