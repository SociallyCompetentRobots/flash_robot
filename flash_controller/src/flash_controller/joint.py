import time


class Joint(object):
    
    
    def __init__(self, urbi_wrapper, name, pos_min, pos_max, raw_zero):
        self.name       = name
        self.uw         = urbi_wrapper
        self.raw_max    = float(self.uw.send(self.name + '.val->rangemax')[0])
        self.raw_min    = float(self.uw.send(self.name + '.val->rangemin')[0])
        self.raw_zero   = raw_zero
        self.pos_min    = pos_min
        self.pos_max    = pos_max
    
    
    def clipPositionLimits(self, position):
        return max(min(self.pos_max, position), self.pos_min)

    
    @property
    def val(self):        
        return float(self.uw.send(self.name + '.val')[0])
    

    @property
    def pos(self):        
        """ """
        return float(self.uw.send(self.name + '.val')[0]) - self.raw_zero
    
    @pos.setter
    def pos(self, value):
        self.uw.send('%s.MoveSpeed(%.4f, 30.0)' % (self.name, self.clipPositionLimits(value)))
        time.sleep(0.5)


    def center(self):
        if self.pos != 0.0:
            self.pos = 0.0


    def move(self, position, time):
        position = self.clipLimits(position)
        self.uw.send(self.name + '.Move(%.4f, %.4f)' % (position, time), time)


    def __str__(self):
        return "%s\t[%.3f, %.3f]\t%.3f" % (self.name, self.pos_min, self.pos_max, self.pos)
