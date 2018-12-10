


class Sensor:
    
    
    def __init__(self, urbi_wrapper, name, rangemin = 0.0, rangemax = 0.0):
        self.name = name
        self.uw   = urbi_wrapper
        self.min  = rangemin
        self.max  = rangemax


    @property
    def value(self):
        return float(self.uw.send(self.name + '.val')[0])


    def __str__(self):
        return "%s\t[%.3f, %.3f]\t%.3f" % (self.name, self.min, self.max, self.value)
