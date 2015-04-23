import os
import os.path
import sys
import socket
import thread
import utili
from datetime import datetime
from collections import deque

# global variabls
g_fname = None
g_sock_kinect = None
g_sock_myo = None
g_diff = 50 #ms

def init_conn ():
    g_sock_kinect = None
    g_sock_myo = None

def exec_line (cmd, startTime):
    
    toks = line.split () 
    etime = int (toks[0])
    ctime = int (round (time.time ()

    # recording format:
    #   time dev data...   
    while abs (ctime - st) - etime) < g_diff:
        print 'exec {' + line + '} @ ' + ctime
        dev = toks[2]

        if dev == 'KNCT':
            break;

        if dev == 'MYO':
            break;

def replay ():

    st = int (datetime.datetime.now () 
    print 'starting time: ' + st

    with open (g_fname) as rec:
        for line in rec :
            exec_line (line, st)

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
