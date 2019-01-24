

class Joint(object):
    """ The Joint class provides an API for individual joints.
    
    The joint values can be read and modified in two different ways. 
    
    1)  The 'pos' values relate to a normalized position range sensible for the joint's function.
        It is the preferred way to manipulate joints and it takes timing into account.

    2)  The 'raw' values relate to the motors own angular position and will be specific to the 
        individual motor.
        Be aware: only use this way if you know what you are doing! Setting the raw values implies
        that no trajectory generator is used and the motor will move as fast as possible to the 
        targeted position. This might not be safe in all cases and might damage the motor.
    """

    
    def __init__(self, urbi_wrapper, name, pos_min, pos_max, raw_zero):
        self.name       = name
        self.uw         = urbi_wrapper
        self.raw_max    = float(self.uw.send(self.name + '.val->rangemax')[0])
        self.raw_min    = float(self.uw.send(self.name + '.val->rangemin')[0])
        self.raw_zero   = raw_zero
        self.pos_min    = pos_min
        self.pos_max    = pos_max
        self.ratio      = round(abs(self.raw_max - self.raw_min) / abs(self.pos_max - self.pos_min), 1)
    
    
    def clipPosLimits(self, pos):
        """ Clips the given position to the min and max values which are given by the configuration.
        
        @param     pos:float - position
        @return    clipped position
        """
        return max(min(self.pos_max, pos), self.pos_min)


    def clipRawLimits(self, raw):
        """ Clips the given position to the min and max values which are read from the motors.
        
        @param     raw:float - raw angle
        @return    clipped position
        """
        return max(min(self.raw_max, raw), self.raw_min)
        
    
    @property
    def raw(self):
        """ Returns the raw motor angle. """
        return float(self.uw.send(self.name + '.val')[0])
    

    @raw.setter
    def raw(self, raw):
        """ Sets the raw motor angle to raw. 
        
        Be aware: this will set the value directly and moves the motor as fast as possible in this
                  position. No trajectory generation is done!
        """
        self.uw.send('%s.val = %.4f' % (self.name, self.clipRawLimits(raw)))


    @property
    def pos(self):
        """ Returns the position angle. """
        return (self.raw - self.raw_zero) / self.ratio


    @pos.setter
    def pos(self, value):
        """ Sets the position angle by using the MoveSpeed method which will generate an
            appropriate trajectory.
        """
        self.uw.send('%s.val = %.4f smooth:2' % (self.name, (self.clipPosLimits(value) * self.ratio) + self.raw_zero))


    def center(self):
        """ Moves the joint to it's center position which is based on the configuration. """
        if self.pos != 0.0:
            self.pos = 0.0


    def move(self, position, time):
        """ Moves the joint in position over time. """
        self.uw.send(self.name + '.Move(%.4f, %.4f)' % (self.clipLimits(position), time), time)


    def __str__(self):
        return "%s\t[%.3f, %.3f, %.3f]\t%.3f\t[%.3f, %.3f]\t%.3f" % ( self.name, 
                                                                      self.raw_min, 
                                                                      self.raw_zero, 
                                                                      self.raw_max,
                                                                      self.ratio, 
                                                                      self.pos_min, 
                                                                      self.pos_max, 
                                                                      self.pos )
