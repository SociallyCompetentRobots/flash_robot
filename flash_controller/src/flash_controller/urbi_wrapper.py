import logging
import sys

from telnetlib  import Telnet
from time       import sleep

LOG = logging.getLogger(__name__)
LOG.setLevel(logging.DEBUG)

ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
LOG.addHandler(ch)

class UrbiWrapper:
    """ The UrbiWrapper class provides a singleton to communicate with an URBI instance via telnet.
    
    Note: To control more than one robot at once this wrapper should be re-implemented to get rid of
    the singleton pattern implementation. But this might complicate the implementation of the users
    of this class.
    """
    HOST = '10.0.0.195'
    PORT = 54000
    ################################################################################################
    # Singleton Pattern Implementation: Start
    ################################################################################################
    __instance = None

    def __new__(cls, *args, **kwargs):
        if UrbiWrapper.__instance is None:
            UrbiWrapper.__instance = super(UrbiWrapper, cls).__new__(cls)
        return UrbiWrapper.__instance

    ################################################################################################
    # Singleton Pattern Implementation: End
    ################################################################################################

    SLEEP = 0.05
    WAIT  = 1.0
    
    HEADER = """*** ********************************************************
*** Urbi version 2.7.5 patch 0 revision 846b3de
*** Copyright (C) 2004-2012 Gostai S.A.S.
***
*** This program comes with ABSOLUTELY NO WARRANTY.  It can
*** be used under certain conditions.  Type `license;',
*** `authors;', or `copyright;' for more information.
***
*** Check our community site: http://www.urbiforge.org.
*** ********************************************************"""


    def __init__(self, host = HOST, port = PORT):
        self.HOST   = host
        self.PORT   = port
        LOG.debug('Open Connection to: %s:%s' % (host, port))

        self.tn     = Telnet()
        self.tn.open(self.HOST, self.PORT)

        # read the header and check that we are connected to the right thing.
        header, _ = self.read_all(2.0)
        header    = header.decode('ascii')
#         print(header)
#         if header is not UrbiWrapper.HEADER:
#             raise RuntimeError('Retrieved URBI header is not correct got\n[%s]\n instead of\n[%s]' % (header, UrbiWrapper.HEADER))
        


    def read_line(self, timeout = WAIT):
        """ Reads a line from the telnet server. A line is a ended by a newline character.

        If no content was read the method returns an empty content string and the timestamp will be
        0.
        
        @return content, timestamp
        """

        timestamp = 0
        content   = ''

        # read until new line character
        line = self.tn.read_until(b'\n', timeout)

        # get rid of the new line character
        line = line[:-1]

        if line:

            # split
            line = line.split(b' ')

            # get timestamp
            try:
                timestamp   = int(line[0][1:-1])
            except ValueError:
                raise ValueError('malformed timestamp in line [%s]' % (b' '.join(line)))

            # merge the rest into content
            content     = b' '.join(line[1:])
        
        return content, timestamp


    def read_all(self, timeout = WAIT):
        """ Reads the output from a telnet server until there is no more for a timeout period."""
        result          = []
        line, timestamp = self.read_line(timeout)
        while line:
            result.append(line)
            line, _ = self.read_line(timeout)
        return b'\n'.join(result), timestamp


    def send(self, cmd, timeout = WAIT):
        """ Sends a URBI command to the server. 
        
        @param: cmd - 
        """
        

        # sanitize the command            
        cmd = cmd + ';' if cmd[-1] is not ';' else cmd
        LOG.debug('send: [' + str(cmd) + ']')

        # encode as ASCII
        self.tn.write(cmd.encode('ascii'))

        # we wait a small while to give the server a chance to react            
        sleep(UrbiWrapper.SLEEP)
        
        # read all within a time frame
        result, timestamp = self.read_line(timeout)

        LOG.debug('received: %s' % result)
        
        return result, timestamp


    def __del__(self):
        self.tn.close()