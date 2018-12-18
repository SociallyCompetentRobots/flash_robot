import logging
import sys
import time

import socket

LOG = logging.getLogger(__name__)
LOG.setLevel(logging.INFO)

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
    HOST  = '10.0.0.195'
    PORT  = 54000
    SLEEP = 0.05
    WAIT  = 0.06
    
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

        self.tn     = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tn.connect((self.HOST, self.PORT))
        time.sleep(0.5)

        # read the header and check that we are connected to the right thing.
        
        header, _ = self.send(' ')
        header    = header.decode('ascii')
#         if header is not UrbiWrapper.HEADER:
#             raise RuntimeError('Retrieved URBI header is not correct got\n[%s]\n instead of\n[%s]' % (header, UrbiWrapper.HEADER))

    
    @property
    def isConnected(self):
        return True


    def read(self):
        
        result      = []
        timestamp   = 0.0
        last_line   = ''

        while last_line != b'"EOF"':
            data        = self.tn.recv(16*1024)
            for line in data.split(b'\n'):
                if not line:
                    continue
                line  = line.split(b' ')
                
                # get timestamp
                try:
                    error       = b'error' in line[0]
                    timestamp   = int(line[0][1:-1] if not error else line[0][1:-7])
                except ValueError:
                    raise ValueError('malformed timestamp in line [%s]' % (b' '.join(line)))
    
                # merge the rest into content                
                last_line = b' '.join(line[1:]) if not error else b'error'
                result.append(last_line)

        return b'\n'.join(result[:-1]), timestamp

    
    def send(self, cmd):
        """ Sends a URBI command to the server. 
        
        @param: cmd - 
        """
        # sanitize the command            
        cmd = cmd + ';' if cmd[-1] is not ';' else cmd

        # add EOF handling
        cmd += '"EOF";'
        
        LOG.debug('send: [' + str(cmd) + ']')

        # encode as ASCII
        self.tn.sendall(cmd.encode('ascii'))

        # read all within a time frame
        result, timestamp = self.read()

        LOG.debug('received: %s' % result)
        
        return result, timestamp


    def __del__(self):
        self.tn.close()