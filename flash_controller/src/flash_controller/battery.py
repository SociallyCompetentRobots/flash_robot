from flash_controller.urbi_wrapper import UrbiWrapper


class Battery:
    """ The Flash class provides the Python API for the FLASH robot. """

    
    def __init__(self):
        """ Creates an Flash API object and connects to the robot. """
        self.uw = UrbiWrapper()

        # check if the result from the dummy request fits; otherwise abort
        if not self.uw.isConnected:
            raise RuntimeError('Connection to Flash failed.')


    @property
    def voltage(self):
        return float(self.uw.send("robot.body.battery.voltage")[0])


    @property
    def reading(self):
        value, timestamp = self.uw.send("robot.body.battery.voltage") 
        return float(value), timestamp
