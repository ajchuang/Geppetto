import os
import os.path
import sys
import socket
import thread
import util
from collections import deque

# global variabls
g_fname = None
g_sock_kinect = None
g_sock_myo = None

def init_conn ():
    g_sock_kinect = None
    g_sock_myo = None

def replay ():
    with open (g_fname) as rec:
        for line in rec :
            print '  [Log] ' + line

# main function
if __name__ == "__main__":
   
    if (len (sys.argv) != 2)
        print '[Error] Please specify recording file correctly'
        print '[Usage] python MrGeppetto_replay.py [recording]'
        return

    g_fname = sys.argv[1]
    
    if not os.path.isfile (g_fname):
        print '[Error] Recording file ' + g_fname + ' does not exist'
        return
    else:
        print 'Replaying file: ' + g_fname

    g_host, g_port = util.read_conf ('recorder');
    
    # starting the server
    replay ()
